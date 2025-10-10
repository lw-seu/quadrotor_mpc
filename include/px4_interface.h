#pragma once

#include <ros/ros.h>
#include <thread>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include "mpc_tracking.h"

class OffboardMode
{
private:
    ros::Rate rate_;
    ros::NodeHandle nh_;
    ros::Publisher bodyrate_thrust_publisher_;
    ros::Publisher pos_publisher_;
    ros::Publisher velocity_publisher_;
    ros::Publisher actuator_publisher_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    ros::Subscriber state_subscriber_;
    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;
    mavros_msgs::AttitudeTarget bodyrate_thrust_;
    mavros_msgs::ActuatorControl torque_thrust_;
    geometry_msgs::PoseStamped pose_;
    geometry_msgs::TwistStamped twist_;
    std::thread set_offboard_mode_thread_;
    std::thread pub_mavros_control_thread_;
    real_t min_thrust_;
    real_t max_thrust_;
    bool is_mpc_start_;

    TrackingMpc* mpc_ros_application_;
    
public:
    OffboardMode(TrackingMpc* mpc_ros_application);
    ~OffboardMode() {}
    // mavros_msgs::AttitudeTarget bodyrate_thrust_;
    int mainLoop();

    void pubMavrosControl();
    void px4CurrentState(const mavros_msgs::State::ConstPtr& msg);
    void setOffboardForSimulation();
    void setLandMode();
    void updateMavrosControl( 
        const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, 1>> control);
    void shutdown();
    float controlNormalization(float signal, float min, float max);
};