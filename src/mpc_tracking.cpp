#include "mpc_tracking.h"

#include <cmath>
#include <iostream>

TrackingMpc::TrackingMpc(): trajectory_type_(TRAJECTORY_LEMNISCATE), 
                                        speed_(3.0),
                                        let_mpc_run_(false),
                                        gamma_(-0.1)
{
    init();
}

TrackingMpc::~TrackingMpc()
{

}

void TrackingMpc::init() 
{
    Eigen::Matrix<real_t, ACADO_NX, 1> initial_states;
    initial_states << 0.0, 0.0, 0.0, 
                      1.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0;
    current_states_ = initial_states;
    predicted_states_ = initial_states.replicate(1, ACADO_N + 1);
    reference_states_.setZero();
    reference_controls_.setZero();
    current_control_.setZero();

    Gamma_ = Eigen::Matrix<double, ACADO_NX, ACADO_NX>::Identity() * gamma_;


    q_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() <<
        10.0 * Eigen::Matrix<real_t, 2, 1>::Ones(),
        5,
        10.0 * Eigen::Matrix<real_t, 4, 1>::Ones(),
        0.1 * Eigen::Matrix<real_t, 3, 1>::Ones()).finished().asDiagonal();
    r_ = (Eigen::Matrix<real_t, ACADO_NU, 1>() <<
        0.1 * Eigen::Matrix<real_t, ACADO_NU, 1>::Ones()).finished().asDiagonal();

    mpc_wrapper_.initMpc(current_states_, predicted_states_, reference_states_, 
        reference_controls_, q_, r_);
    mpc_wrapper_.getMpcBound(control_lower_bound_, control_upper_bound_);

    // odom_subscriber_ = nh_.subscribe("/mavros/local_position/odom", 
    //             1, &TrackingMpc::odomCallback, this);

    gdt_subscriber_ = nh_.subscribe("/gazebo/model_states",
                1, &TrackingMpc::gdtCallback, this);
    trajectory_creation_thread_ = std::thread(&TrackingMpc::createTrajectory, this);
}
void TrackingMpc::setReference() 
{
    reference_controls_ = (Eigen::Matrix<real_t, ACADO_NU, 1>() << 
        current_target_states_.target_wx, current_target_states_.target_wy,
        current_target_states_.target_wz,
        current_target_states_.target_thrust ).finished().replicate(1, ACADO_N);
    // 调用MpcWrapper函数将reference传入到mpc_wrapper_中
    mpc_wrapper_.setMpcReference(reference_states_, reference_controls_);
}

void TrackingMpc::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) 
{
    int i = 0;
    double current_time = odom_msg->header.stamp.toSec();
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.position.x);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.position.y);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.position.z);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.orientation.w);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.orientation.x);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.orientation.y);
    current_states_[i++] = static_cast<real_t>(odom_msg->pose.pose.orientation.z);
    current_states_[i++] = static_cast<real_t>(odom_msg->twist.twist.linear.x);
    current_states_[i++] = static_cast<real_t>(odom_msg->twist.twist.linear.y);
    current_states_[i++] = static_cast<real_t>(odom_msg->twist.twist.linear.z);
    current_states_.segment(3,4).normalize();

    if(current_states_.segment(3,4).dot(reference_states_.col(0).segment(3,4)) < 0)
    {
        current_states_.segment(3,4) = -current_states_.segment(3,4);
        Eigen::Matrix<real_t, 4, 1> q = current_states_.segment(3,4);
    }

    is_new_data_ = true;
    mpc_count_ = 0;
}

void TrackingMpc::gdtCallback(const gazebo_msgs::ModelStatesConstPtr& gdt_msg)
{
    int i = 0; 
    for(int j = 0; j < gdt_msg->name.size(); j++)
    {
        if(gdt_msg->name[j] == "iris")
        {
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].position.x);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].position.y);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].position.z);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].orientation.w);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].orientation.x);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].orientation.y);
            current_states_[i++] = static_cast<real_t>(gdt_msg->pose[j].orientation.z);
            current_states_[i++] = static_cast<real_t>(gdt_msg->twist[j].linear.x);
            current_states_[i++] = static_cast<real_t>(gdt_msg->twist[j].linear.y);
            current_states_[i++] = static_cast<real_t>(gdt_msg->twist[j].linear.z);
            current_states_.segment(3,4).normalize();

            if(current_states_.segment(3,4).dot(reference_states_.col(0).segment(3,4)) < 0)
            {
                current_states_.segment(3,4) = -current_states_.segment(3,4);
                Eigen::Matrix<real_t, 4, 1> q = current_states_.segment(3,4);
            }

            Eigen::Quaterniond q(current_states_(3), current_states_(4), 
                current_states_(5), current_states_(6));
            
            attitude_ = Quaternion2Euler(q);

            if(trajectory_type_!=TRAJECTORY_NONE && is_at_startposition_ && !is_contact_)
            {
                static auto start_time = std::chrono::steady_clock::now();
                auto current_time = std::chrono::steady_clock::now();
                dataLog(std::chrono::duration<double>(current_time - start_time).count());
            }
            is_new_data_ = true;
            mpc_count_ = 0;

            break;
        }
    }
}

void TrackingMpc::setWeight()
{
   
}

void TrackingMpc::setOnlineData()
{
    mpc_wrapper_.setMpcOnlineData(online_data_);
}

void TrackingMpc::mpcProcess()
{
    while (ros::ok())
    {

        static auto current_time = std::chrono::steady_clock::now();

        if(is_contact_)
        {
            mpc_error_ = CONTACT_DETECTED;
            return;
        }

        auto pre_time = current_time;
        current_time = std::chrono::steady_clock::now();

        Eigen::VectorXd y = Eigen::VectorXd::Zero(ACADO_NX);
        y << current_states_(0), current_states_(1), current_states_(2),
            current_states_(3), current_states_(4), current_states_(5),
            current_states_(6), current_states_(7), current_states_(8),
            current_states_(9);
        if(is_first_estimation_)
        {
            is_first_estimation_ = false;
            Eigen::VectorXd z = Eigen::VectorXd::Zero(ACADO_NX);
            z << y; 
            disturbance_observer_.setFirstZ(z);
        }
        else
        {
            disturbance_observer_.setDt(
                std::chrono::duration<double>(current_time - pre_time).count());
        }
        
        Eigen::VectorXd u = Eigen::VectorXd::Zero(ACADO_NU);
        u << current_control_(0), current_control_(1), current_control_(2), current_control_(3);
        // disturbance_observer_.update(y, u, current_states_(5), current_states_(6), current_states_(7), kMass, kG, visual_jacob);
        // disturbance_observer_.finite_update(y, u, current_states_(5), current_states_(6), current_states_(7), kMass, kG, visual_jacob);
        // disturbance_observer_.nonlinear_update(y, u, current_states_(5), current_states_(6), current_states_(7), kMass, kG, visual_jacob);
        disturbance_observer_.kalman_geso_update(y, u, kMass, kG);  // 干扰估计
        d_hat_.block<3,1>(0,0) = disturbance_observer_.getDisturbance().block<3, 1>(0, 0).cast<real_t>();

        online_data_ = (Eigen::Matrix<real_t, ACADO_NOD, 1>() << d_hat_.block<3,1>(0,0)).finished().replicate(1, ACADO_N + 1);
        // printf("onlinedata: %f\t %f\t %f\n", online_data_(0,0) ,online_data_(1,0), online_data_(2,0));
        setReference();

        setWeight();

        setOnlineData();

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); 
        std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - current_time);
        double period = duration.count();

        if(is_new_data_)
        {
            is_new_data_ = false;

            // std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            {
                // std::lock_guard<std::mutex> lock(mpc_mutex_);
                if(mpc_wrapper_.solveMpc(current_states_))
                {
                    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); 
                    // std::chrono::milliseconds duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                    // double period = duration.count();
                    // std::cout << "mpcsolve time: " << period << " ms" << std::endl;         
                    mpc_wrapper_.getMpcControl(current_control_, 0);
                    // std::cout << "mpc_control:" << current_control_.transpose() << std::endl;
                }
                else
                {
                    ROS_WARN("MPC solve failed!");
                }
            }
            
        }
        else
        {
            mpc_count_++;
            if(mpc_count_ >= ACADO_N)
            {
                mpc_error_ = DATA_LOST;
            }
            else
            {
                mpc_wrapper_.getMpcControl(current_control_, mpc_count_);
            }
        }  
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

Eigen::Vector3d TrackingMpc::Quaternion2Euler(Eigen::Quaterniond q)
{
    Eigen::Vector3d euler(0, 0, 0);
    euler(0) = atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                    q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
    euler(1) = asin(-2 * (q.x() * q.z() - q.w() * q.y()));
    euler(2) = atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                   q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return euler;
}

Eigen::Quaternion<real_t> TrackingMpc::euler2Quaternion(double roll, double pitch, double yaw)
{
    Eigen::Quaternion<real_t> q;
    real_t r, p, y;
    r = roll/2.0;
    p = pitch/2.0;
    y = yaw/2.0;
    q.w() = cos(r) * cos(p) * cos(y) + sin(r) * sin(p) * sin(y);
    q.x() = sin(r) * cos(p) * cos(y) - cos(r) * sin(p) * sin(y);
    q.y() = cos(r) * sin(p) * cos(y) + sin(r) * cos(p) * sin(y);
    q.z() = cos(r) * cos(p) * sin(y) - sin(r) * sin(p) * cos(y);
    return q; 
}

std::vector<Point2D> TrackingMpc::generateLemniscateTrajectory(double a, double t_step) {
    std::vector<Point2D> trajectory;

    // 生成 3 圈，从中间点开始
    double start_t = -M_PI / 2.0;       // 从交叉点开始
    double end_t   = start_t + 3 * 2 * M_PI;  // 三圈

    int num_points = static_cast<int>((end_t - start_t) / t_step);

    for (int i = 0; i <= num_points; ++i) {
        double t = start_t + i * t_step;
        double denom = 1 + std::pow(std::sin(t), 2);
        double x = a * std::cos(t) / denom;
        double y = a * std::cos(t) * std::sin(t) / denom;
        trajectory.push_back({x, y});
    }

    return trajectory;
}

std::vector<double> TrackingMpc::computeArcLength(const std::vector<Point2D>& traj) {
    std::vector<double> arc_len;
    arc_len.push_back(0.0);
    for (size_t i = 1; i < traj.size(); ++i) {
        double dx = traj[i].x - traj[i - 1].x;
        double dy = traj[i].y - traj[i - 1].y;
        double ds = std::sqrt(dx * dx + dy * dy);
        arc_len.push_back(arc_len.back() + ds);
    }
    return arc_len;
}

void TrackingMpc::createTrajectory()
{
    while (ros::ok())
    {
        if (!is_at_startposition_)
        {
            while (!let_mpc_run_)
            {
                /* code */
            }
            static double target_x = 0.0;           // 缓启动
            static double target_y = 0.0;
            static double target_z = 0.0;
            static int target_count = 0;
            int resolution = 300;

            if(target_count < resolution)
            {
                target_x += current_target_states_.target_x / static_cast<double>(resolution);
                target_y += current_target_states_.target_y / static_cast<double>(resolution);
                target_z += current_target_states_.target_z / static_cast<double>(resolution);
                target_count++;
            }
            
            float horizontal_distance = 0.0;
            float z_distance = 0.0;
            horizontal_distance = 
                sqrtf(powf(current_states_[0]- current_target_states_.target_x, 2) + 
                    powf(current_states_[1] - current_target_states_.target_y, 2));
            z_distance = current_states_[2];
            
            reference_states_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
            target_x, target_y, target_z, 
            current_target_states_.target_q.w(), current_target_states_.target_q.x(),
            current_target_states_.target_q.y(), current_target_states_.target_q.z(),
            current_target_states_.target_vx, current_target_states_.target_vy, 
            current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1);
            if(horizontal_distance < 0.1 && z_distance > current_target_states_.target_z - 0.1)
            {
                std::this_thread::sleep_for(std::chrono::seconds(6));
                // trajectory_type_ = TRAJECTORY_CIRCLE;
                // trajectory_type_ = TRAJECTORY_CIRCLE_2;
                // trajectory_type_ = TRAJECTORY_LEMNISCATE;
                if(trajectory_type_ == TRAJECTORY_LEMNISCATE)
                {
                    raw_traj_ = generateLemniscateTrajectory(lemniscate_a_, 0.001);
                    arc_len_ = computeArcLength(raw_traj_);
                }
                is_at_startposition_ = true;
                // let_mpc_run_ = true;
            }
        } 
        else
        {
            switch (trajectory_type_)
            {
                case TRAJECTORY_NONE:
                {
                    reference_states_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                    current_target_states_.target_x, current_target_states_.target_y,
                    current_target_states_.target_z, 
                    current_target_states_.target_q.w(), current_target_states_.target_q.x(),
                    current_target_states_.target_q.y(), current_target_states_.target_q.z(),
                    current_target_states_.target_vx, current_target_states_.target_vy, 
                    current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1);
                    break;
                }
                    
                case TRAJECTORY_CIRCLE:  // 平面圆，z不变
                {
                    static auto start_time = std::chrono::steady_clock::now();
                    static double r = 1.0;
                    static int count = 0;
                    auto now = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now - start_time).count();
                    double theta = speed_ * elapsed / r;  // 弧度单位

                    // 飞行三圈判断（从 0 到 6π）
                    if (theta >= (count + 1) * 2 * M_PI) {
                        count++;
                    }

                    // 若已完成 3 圈，可以触发某些控制逻辑
                    if (count >= 3) {
                        current_target_states_.target_x = current_target_states_.kStartPosX;
                        current_target_states_.target_y = current_target_states_.kStartPosY;
                        current_target_states_.target_z = current_target_states_.kStartPosZ;
                        // let_mpc_run_ = false;
                        trajectory_type_ = TRAJECTORY_NONE;
                        reference_states_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                        current_target_states_.target_x, current_target_states_.target_y,
                        current_target_states_.target_z, 
                        current_target_states_.target_q.w(), current_target_states_.target_q.x(),
                        current_target_states_.target_q.y(), current_target_states_.target_q.z(),
                        current_target_states_.target_vx, current_target_states_.target_vy, 
                        current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1);
                        break;
                    }

                    // 圆轨迹坐标更新
                    current_target_states_.target_x = r - r * std::cos(theta)-1.0;
                    current_target_states_.target_y = r * std::sin(theta);

                    for(int i = 0; i < ACADO_N + 1; i++)
                    {
                        double theta_i = theta + speed_ * mpc_time_step_ * i / r;
                        if(count >= 2 && theta_i >= 6*M_PI)
                        {
                            current_target_states_.target_x = current_target_states_.kStartPosX;
                            current_target_states_.target_y = current_target_states_.kStartPosY;
                            current_target_states_.target_z = current_target_states_.kStartPosZ;
                            reference_states_.block(0,i,kCostStateSize,ACADO_N + 1-i) = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                            current_target_states_.target_x, current_target_states_.target_y, current_target_states_.target_z, 
                            current_target_states_.target_q.w(), current_target_states_.target_q.x(),
                            current_target_states_.target_q.y(), current_target_states_.target_q.z(),
                            current_target_states_.target_vx, current_target_states_.target_vy, 
                            current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1-i);
                            break;
                        }
                        else
                        {
                            reference_states_.block(0,i,kCostStateSize,1) = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                            r - r * std::cos(theta_i)-1.0, 
                            r * std::sin(theta_i),
                            current_target_states_.target_z,
                            current_target_states_.target_q.w(),
                            current_target_states_.target_q.x(),
                            current_target_states_.target_q.y(),
                            current_target_states_.target_q.z(),
                            current_target_states_.target_vx,
                            current_target_states_.target_vy,
                            current_target_states_.target_vz).finished();
                        }
                        
                    }
                    // current_target_states_.target_vx = speed_ * std::sin(theta);
                    // current_target_states_.target_vy = speed_ * std::cos(theta);
                    break;
                }

                case TRAJECTORY_CIRCLE_2: // 3D圆，z变化
                {
                    static auto start_time = std::chrono::steady_clock::now();
                    static double r = 3.0;
                    static int count = 0;
                    static double center_x = r;
                    static double center_y = 0.0;
                    static double center_z = 1.5;
                    auto now = std::chrono::steady_clock::now();
                    double elapsed = std::chrono::duration<double>(now - start_time).count();
                    double theta = speed_ * elapsed / r;

                    if (theta >= (count + 1) * 2 * M_PI) {
                        count++;
                    }

                    if (count >= 3) {
                        current_target_states_.target_x = current_target_states_.kStartPosX;
                        current_target_states_.target_y = current_target_states_.kStartPosY;
                        current_target_states_.target_z = current_target_states_.kStartPosZ;
                        trajectory_type_ = TRAJECTORY_NONE;
                        reference_states_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                        current_target_states_.target_x, current_target_states_.target_y,
                        current_target_states_.target_z, 
                        current_target_states_.target_q.w(), current_target_states_.target_q.x(),
                        current_target_states_.target_q.y(), current_target_states_.target_q.z(),
                        current_target_states_.target_vx, current_target_states_.target_vy, 
                        current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1);
                        break;
                    }

                    current_target_states_.target_x = r - r * std::cos(theta);
                    current_target_states_.target_y = r * std::sin(theta);
                    current_target_states_.target_z = 1 + 0.2 * current_target_states_.target_x;
                    // double pitch = std::atan((center_z - current_target_states_.target_z)/r);
                    double pitch = 0;
                    double yaw = -theta;
                    // if(static_cast<int>(std::trunc(theta / M_PI))  % 2 == 0)
                    // {
                    //     yaw = -fmod(theta, M_PI);
                    // }
                    // else 
                    // {
                    //     yaw = M_PI*2-fmod(theta, M_PI*2);
                    // }
                    
                    current_target_states_.target_euler(0) = 0;
                    current_target_states_.target_euler(1) = pitch;
                    if(static_cast<int>(std::trunc(theta / M_PI))  % 2 == 0)
                    {
                        current_target_states_.target_euler(2) = -fmod(theta, M_PI);
                    }
                    else 
                    {
                        current_target_states_.target_euler(2) = M_PI*2-fmod(theta, M_PI*2);
                    }
                    // current_target_states_.target_euler(2) = yaw;
                    Eigen::Quaternion<real_t> q = euler2Quaternion(0, pitch, yaw);
                    q.normalize();
                    current_target_states_.target_q.w() = q.w();
                    current_target_states_.target_q.x() = q.x();
                    current_target_states_.target_q.y() = q.y();
                    current_target_states_.target_q.z() = q.z();
                    for(int i = 0; i < ACADO_N + 1; i++)
                    {
                        double theta_i = theta + speed_ * mpc_time_step_ * i / r;
                        if(count >= 2 && theta_i >= 6*M_PI)
                        {
                            reference_states_.block(0,i,kCostStateSize,ACADO_N + 1-i) = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                            0 , 0, 1.0, 
                            current_target_states_.target_q.w(), current_target_states_.target_q.x(),
                            current_target_states_.target_q.y(), current_target_states_.target_q.z(),
                            current_target_states_.target_vx, current_target_states_.target_vy, 
                            current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1-i);
                            break;
                        }
                        else
                        {                            
                            // pitch = std::atan((center_z - 1 + 0.2 * (r - r * std::cos(theta_i)))/r);
                            pitch = 0;
                            yaw = -(theta_i);
                            q = euler2Quaternion(0, pitch, yaw);
                            q.normalize();
                            reference_states_.block(0,i,kCostStateSize,1) = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                            r - r * std::cos(theta_i), 
                            r * std::sin(theta_i),
                            1 + 0.2 * (r - r * std::cos(theta_i)),
                            q.w(),
                            q.x(),
                            q.y(),
                            q.z(),
                            current_target_states_.target_vx,
                            current_target_states_.target_vy,
                            current_target_states_.target_vz).finished();
                        }
                    }
                    break;
                }     
                case TRAJECTORY_LEMNISCATE: // r^2 = 2a^2cos(2theta)
                    {
                        static auto start_time = std::chrono::steady_clock::now();
                        static double target_s = 0.0;
                        // static int count = 0;  // 计数器，用于判断是否完成一圈
                        static size_t j = 0;
                        // static double center_x = r;
                        // static double center_y = 0.0;
                        // static double center_z = 1.5;
        
                        auto now = std::chrono::steady_clock::now();
                        double elapsed = std::chrono::duration<double>(now - start_time).count();
                        target_s = speed_ * elapsed;
                        
                        // if(target_s >= 7.416/sqrt(2)*lemniscate_a_*3)
                        // {
                        //     reference_states_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                        //     lemniscate_a_ , 0, current_target_states_.target_z, 
                        //     current_target_states_.target_q.w(), current_target_states_.target_q.x(),
                        //     current_target_states_.target_q.y(), current_target_states_.target_q.z(),
                        //     current_target_states_.target_vx, current_target_states_.target_vy, 
                        //     current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1);
                        //     return;
                        // }
                        current_target_states_.target_x = raw_traj_[j].x;
                        current_target_states_.target_y = raw_traj_[j].y;
                        if (j >= arc_len_.size() - 1) 
                        {
                            reference_states_ = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                            raw_traj_[arc_len_.size() - 1].x , raw_traj_[arc_len_.size() - 1].y, current_target_states_.target_z, 
                            current_target_states_.target_q.w(), current_target_states_.target_q.x(),
                            current_target_states_.target_q.y(), current_target_states_.target_q.z(),
                            current_target_states_.target_vx, current_target_states_.target_vy, 
                            current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1);
                            trajectory_type_ = TRAJECTORY_NONE;
                            return;
                        }
                        // double ratio = (target_s - arc_len_[j]) / (arc_len_[j + 1] - arc_len_[j]);
                        // double x = raw_traj_[j].x + ratio * (raw_traj_[j + 1].x - raw_traj_[j].x);
                        // double y = raw_traj_[j].y + ratio * (raw_traj_[j + 1].y - raw_traj_[j].y);

                        size_t temp_j = j;
                        for(int i = 0; i < ACADO_N + 1; i++)
                        {
                            double target_s_i = target_s + speed_ * mpc_time_step_ * i;
                            // if(target_s >= 7.416/sqrt(2)*lemniscate_a_*3)
                            // {
                            //     reference_states_.block(0,i,kCostStateSize,ACADO_N + 1-i) = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                            //     lemniscate_a_ , 0, current_target_states_.target_z, 
                            //     current_target_states_.target_q.w(), current_target_states_.target_q.x(),
                            //     current_target_states_.target_q.y(), current_target_states_.target_q.z(),
                            //     current_target_states_.target_vx, current_target_states_.target_vy, 
                            //     current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1-i);
                                
                            //     break;
                            // }
                                
                            if (j >= arc_len_.size() - 1) 
                            {
                                reference_states_.block(0,i,kCostStateSize,ACADO_N + 1-i) = (Eigen::Matrix<real_t, kCostStateSize, 1>() << 
                                raw_traj_[arc_len_.size() - 1].x , raw_traj_[arc_len_.size() - 1].y, current_target_states_.target_z, 
                                current_target_states_.target_q.w(), current_target_states_.target_q.x(),
                                current_target_states_.target_q.y(), current_target_states_.target_q.z(),
                                current_target_states_.target_vx, current_target_states_.target_vy, 
                                current_target_states_.target_vz).finished().replicate(1, ACADO_N + 1-i);
                                break;
                            }
                            double ratio = (target_s - arc_len_[j]) / (arc_len_[j + 1] - arc_len_[j]);
                            reference_states_.block(0,i,kCostStateSize,1) = 
                            (Eigen::Matrix<real_t, kCostStateSize, 1>() <<
                            raw_traj_[j].x + ratio * (raw_traj_[j + 1].x - raw_traj_[j].x),
                            raw_traj_[j].y + ratio * (raw_traj_[j + 1].y - raw_traj_[j].y),
                            current_target_states_.target_z,
                            current_target_states_.target_q.w(),
                            current_target_states_.target_q.x(),
                            current_target_states_.target_q.y(),    
                            current_target_states_.target_q.z(),
                            current_target_states_.target_vx,
                            current_target_states_.target_vy,
                            current_target_states_.target_vz).finished();

                             while (j < arc_len_.size() - 1 && arc_len_[j + 1] < target_s)
                            {
                                ++j;
                            }
                        }
                        j = temp_j;
                        break;
                    }
                default:
                    break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}



Eigen::Vector3f TrackingMpc::getTargetPosition()
{
    return Eigen::Vector3f(current_target_states_.target_x, 
        current_target_states_.target_y, current_target_states_.target_z);
}


void TrackingMpc::dataLog(double time)
{
    Eigen::VectorXd pos_data = Eigen::VectorXd::Zero(18);

    pos_data << current_states_(0), current_states_(1), current_states_(2),
                current_target_states_.target_x,
                current_target_states_.target_y, current_target_states_.target_z,
                current_states_(0)-current_target_states_.target_x,
                current_states_(1)-current_target_states_.target_y,
                current_states_(2)-current_target_states_.target_z,
                attitude_(0)-current_target_states_.target_euler(0),
                attitude_(1)-current_target_states_.target_euler(1),
                attitude_(2)-current_target_states_.target_euler(2),
                attitude_(0), attitude_(1), attitude_(2),
                current_target_states_.target_euler(0),
                current_target_states_.target_euler(1), current_target_states_.target_euler(2);

                
    pos_logger_.logData(time, pos_data);
}