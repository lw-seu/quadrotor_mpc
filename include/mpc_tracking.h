#pragma once

#include <deque>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ContactsState.h>
#include <geometry_msgs/Wrench.h>
#include <mutex>
#include "mpc_ros_application.h"
#include "disturbance_observer.h"
#include "plot.h"

// 轨迹点结构
struct Point2D {
    double x;
    double y;
};

class TrackingMpc: public MpcRosApplication 
{
private:
    ros::Subscriber odom_subscriber_;
    ros::Subscriber gdt_subscriber_;
    ros::Subscriber contact_subscriber_;
    ros::Publisher payload_force_publisher_;
    Eigen::MatrixXd qrs_;
    double gamma_;
    Eigen::MatrixXd Gamma_ = Eigen::MatrixXd::Zero(ACADO_NX, ACADO_NX);
    Eigen::MatrixXd ESKF_A_ = Eigen::MatrixXd::Zero(ACADO_NX + ACADO_NOD -1, ACADO_NX + ACADO_NOD -1);

    Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1> online_data_ = 
        Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1>::Zero();

    double mpc_time_step_ = 0.1;
    int mpc_steps_ = 20;
    Eigen::MatrixXd mpc_Bd_ = Eigen::MatrixXd::Zero(ACADO_NX, 3);
    Eigen::MatrixXd mpc_A_ = Eigen::MatrixXd::Zero(ACADO_NX, ACADO_NX);
    Eigen::MatrixXd mpc_P_ = Eigen::MatrixXd::Zero(ACADO_NX, ACADO_NX);

    std::thread trajectory_creation_thread_;
 

    std::mutex mpc_mutex_;

    Eigen::Vector3d attitude_;

    enum TrajectoryType
    {
        TRAJECTORY_NONE = 0,
        TRAJECTORY_CIRCLE,
        TRAJECTORY_CIRCLE_2,
        TRAJECTORY_LEMNISCATE, 
    };

    enum ForcePubType
    {
        FORCE_PUBLISH_NONE = 0,
        FORCE_PUBLISH_ZERO,
        FORCE_PUBLISH_PAYLOAD,
        FORCE_PUBLISH_MINUS_PAYLOAD,
    };

    float payload_force_ = 0.2 * kG; // 负载力，N

    double lemniscate_a_ = 3.0; // 半长轴

    double speed_;

    struct CurrentTargetStates
    {
        real_t target_x;
        real_t target_y;
        real_t target_z;
        Eigen::Quaternion<real_t> target_q;
        Eigen::Vector3d target_euler;
        real_t target_vx;
        real_t target_vy;
        real_t target_vz;
        real_t target_thrust;
        real_t target_wx;
        real_t target_wy;
        real_t target_wz;

        static constexpr real_t kStartPosX = 0.0;
        static constexpr real_t kStartPosY = 0.0;
        static constexpr real_t kStartPosZ = 1.0;
        CurrentTargetStates(): target_x(kStartPosX), target_y(kStartPosY), 
                              target_z(kStartPosZ), target_vx(0.0), 
                              target_vy(0.0), target_vz(0.0), target_thrust(kG*kMass),
                              target_wx(0.0), target_wy(0.0), target_wz(0.0)
        {
            target_q.w() = 1.0;
            target_q.x() = 0.0;
            target_q.y() = 0.0;
            target_q.z() = 0.0;
        }
    };
    bool is_first_estimation_ = true;
    bool is_contact_ = false;
    bool is_at_startposition_ = false;
    DisturbanceObserver disturbance_observer_{0.01};
    CurrentTargetStates current_target_states_;
    TrajectoryType trajectory_type_;
    ForcePubType force_pub_type_ = FORCE_PUBLISH_NONE;
    DataLogger pos_logger_{"~/data/quadrotor_mpc/pos.csv"};

    Eigen::VectorX<real_t> d_hat_ = Eigen::VectorX<real_t>::Zero(3);
    Eigen::VectorX<real_t> dd_hat_ = Eigen::VectorX<real_t>::Zero(3);
    Eigen::VectorX<real_t> pre_d_hat_ = Eigen::VectorX<real_t>::Zero(3);

    std::vector<Point2D> raw_traj_;
    std::vector<double> arc_len_;


public:
    TrackingMpc();
    ~TrackingMpc();

    bool let_mpc_run_;

    void init() override;
    void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
    void gdtCallback(const gazebo_msgs::ModelStatesConstPtr& gdt_msg);
    void setReference() override;
    void setWeight() override;
    void setOnlineData() override;
    void mpcProcess() override;
    void createTrajectory();
    Eigen::Vector3f getTargetPosition();
    Eigen::Quaternion<real_t> euler2Quaternion(double roll, double pitch, double yaw);
    void dataLog(double time);
    Eigen::Vector3d Quaternion2Euler(Eigen::Quaterniond q);
    std::vector<Point2D> generateLemniscateTrajectory(double a, double t_step);
    std::vector<double> computeArcLength(const std::vector<Point2D>& traj);
};
