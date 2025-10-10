#include "mpc_ros_application.h"

void MpcRosApplication::mpcProcess()
{
    while (ros::ok())
    {
        setOnlineData();
        setReference();
        setWeight();
        setParameter();
        setInputBounds();
        // Eigen::Matrix<real_t, ACADO_NU, 1> mpc_control;

        if(is_new_data_)
        {
            is_new_data_ = false;
            if(mpc_wrapper_.solveMpc(current_states_))
            {           
                mpc_wrapper_.getMpcControl(current_control_, 0);
                // std::cout << "mpc_control:" << current_control_.transpose() << std::endl;
            }
            else
            {
                ROS_WARN("MPC solve failed!");
            }
        }
        else
        {
            mpc_count_++;
            if(mpc_count_ > ACADO_N)
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

void MpcRosApplication::mpcStart()
{
    mpc_thread_ = std::thread(&MpcRosApplication::mpcProcess, this);  
}

void MpcRosApplication::mpcOff()
{
    if(mpc_thread_.joinable())
        mpc_thread_.join();
}
