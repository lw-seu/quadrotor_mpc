#include "px4_interface.h"

OffboardMode::OffboardMode(TrackingMpc* mpc_ros_application):
            nh_(),
            rate_(20.0),
            mpc_ros_application_(mpc_ros_application),
            is_mpc_start_(false),
            min_thrust_(0.0),
            max_thrust_(0.0)               
{
    bodyrate_thrust_publisher_ = nh_.advertise<mavros_msgs::AttitudeTarget>
        ("/mavros/setpoint_raw/attitude", 1);
    // set a safe defult thrust value
    bodyrate_thrust_.type_mask = bodyrate_thrust_.IGNORE_ATTITUDE;
    bodyrate_thrust_.body_rate.x = 0.0;
    bodyrate_thrust_.body_rate.y = 0.0;
    bodyrate_thrust_.body_rate.z = 0.0;
    bodyrate_thrust_.thrust = 0.1;

 
    // torque_thrust_.group_mix = 0;
    // torque_thrust_.controls[0] = 0.0;
    // torque_thrust_.controls[1] = 0.0;
    // torque_thrust_.controls[2] = 0.0;
    // torque_thrust_.controls[3] = 0.0;

    
    // actuator_publisher_ = nh_.advertise<mavros_msgs::ActuatorControl>
    //     ("/mavros/actuator_control", 1);
    
    pos_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 1);
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    state_subscriber_ = nh_.subscribe<mavros_msgs::State>
        ("/mavros/state", 10, &OffboardMode::px4CurrentState, this);
    
}

void OffboardMode::shutdown()
{
    ros::waitForShutdown();
    set_offboard_mode_thread_.join();
    pub_mavros_control_thread_.join();
    
    delete mpc_ros_application_;
}

int OffboardMode::mainLoop()
{
    ros::AsyncSpinner spinner(0);
    spinner.start();
    // wait for vehicle connection
    while(ros::ok() && !current_state_.connected)
    { 

    }
    Eigen::Matrix<real_t, ACADO_NU, 1> max_control, min_control;
    max_control = mpc_ros_application_->getControlUpperBound();
    min_control = mpc_ros_application_->getControlLowerBound();


    min_thrust_ = min_control(ACADO_NU-1);
    max_thrust_ = max_control(ACADO_NU-1);

    ROS_INFO("min thrust: %f, max thrust: %f", min_thrust_, max_thrust_);

    offb_set_mode_.request.custom_mode = "OFFBOARD";
    arm_cmd_.request.value = true;

    set_offboard_mode_thread_ = std::thread(&OffboardMode::setOffboardForSimulation, this);
    pub_mavros_control_thread_ = std::thread(&OffboardMode::pubMavrosControl, this);

    shutdown();

    return 0;
}

void OffboardMode::setOffboardForSimulation()
{

    while(ros::ok())
    {
        if( !current_state_.armed )
        {
            if( arming_client_.call(arm_cmd_) &&
                arm_cmd_.response.success)
            {
                ROS_INFO("Vehicle armed.");
                if( current_state_.mode != "OFFBOARD" )
                {
                    if( set_mode_client_.call(offb_set_mode_) &&
                        offb_set_mode_.response.mode_sent)
                    {
                        if(!mpc_ros_application_->let_mpc_run_)
                        {
                            mpc_ros_application_->let_mpc_run_ = true;
                        }
                        ROS_INFO("Offboard mode enabled.");
                    }
                } 
            }
        }

        // if(current_state_.armed)
        // {
        //     if( current_state_.mode != "OFFBOARD" )
        //     {
        //         if( set_mode_client_.call(offb_set_mode_) &&
        //             offb_set_mode_.response.mode_sent)
        //         {
        //             if(!mpc_ros_application_->let_mpc_run_)
        //             {
        //                 mpc_ros_application_->let_mpc_run_ = true;
        //             }
        //             ROS_INFO("Offboard mode enabled.");
        //         }
        //     } 
        // }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    } 
}

void OffboardMode::pubMavrosControl()
{
    while(ros::ok())
    {
        // ROS_INFO("Publishing bodyrate trust");
        // ROS_INFO_THROTTLE(2, "Publishing bodyrate trust");
        MpcError px4_error;
        px4_error = mpc_ros_application_->getMpcError();
        if(is_mpc_start_)
        {
            switch (px4_error)
            {
            case MPC_OK:
                break;
            
            case DATA_LOST:
                {
                    ROS_ERROR("Data lost! Exit.");
                    ros::shutdown();
                }
                break;
            case CONTACT_DETECTED:
                {
                    ROS_WARN("Contact detected! Exit.");
                    ros::shutdown();
                }
                break;
            default:
                break;
            }
        }
        
        updateMavrosControl(mpc_ros_application_->getCurrentControl());
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); 
    }
}

void OffboardMode::updateMavrosControl( 
    const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, 1>> control)
{
    ros::Time current_time = ros::Time::now();
    if(mpc_ros_application_->let_mpc_run_)
    {
        if(is_mpc_start_ == false)
        {
            mpc_ros_application_->mpcStart();
            is_mpc_start_ = true;
        }
    
        bodyrate_thrust_.body_rate.x = control(0);
        bodyrate_thrust_.body_rate.y = control(1);
        bodyrate_thrust_.body_rate.z = control(2);
        bodyrate_thrust_.thrust = controlNormalization(control(3), min_thrust_, max_thrust_);
        bodyrate_thrust_.header.stamp = current_time;
        bodyrate_thrust_publisher_.publish(bodyrate_thrust_);
        // printf("Thrust: %f, Bodyrate: %f, %f, %f\n", bodyrate_thrust_.thrust,
        //     bodyrate_thrust_.body_rate.x, 
        //     bodyrate_thrust_.body_rate.y, 
        //     bodyrate_thrust_.body_rate.z);
    }
    else
    {
        if(is_mpc_start_ == true)
        {
            mpc_ros_application_->mpcOff();
            is_mpc_start_ = false;
        }
        // pose_.header.stamp = current_time;
        // Eigen::Vector3f target_position;
        // target_position = mpc_ros_application_->getTargetPosition();
        // pose_.pose.position.x = target_position(0);
        // pose_.pose.position.y = target_position(1);
        // pose_.pose.position.z = target_position(2);
        // pos_publisher_.publish(pose_);
    }
}

void OffboardMode::px4CurrentState(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
    // ROS_INFO("Recive px4 current state...");
}

void OffboardMode::setLandMode()
{
    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";
    while(current_state_.mode != "AUTO.LAND")
    {
        if(set_mode_client_.call(land_set_mode) &&
            land_set_mode.response.mode_sent)
        {
            ROS_INFO("auto land mode enabled");
        }
    }
}

float OffboardMode::controlNormalization(float signal, float min, float max)
{
    return (signal - min)/(max - min);
}

