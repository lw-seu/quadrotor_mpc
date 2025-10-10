#pragma once

#include "mpc_wrapper.h"
#include <ros/ros.h>
#include <thread>

enum MpcError {
    MPC_OK = 0,
    DATA_LOST ,
    CONTACT_DETECTED,
};

struct Bounds
{
    real_t lower_bound;
    real_t upper_bound;
};

static constexpr float kG = 9.8066;
static constexpr float kMass = 1.5;
// MpcRosApplication is an abstract base class, not allow to create object 
// of this class, only allow to inherit it

class MpcRosApplication
{
protected:
    ros::NodeHandle nh_;
    std::thread mpc_thread_;
    MpcWrapper mpc_wrapper_;

    Eigen::Matrix<real_t, ACADO_NX, 1> current_states_;
    Eigen::Matrix<real_t, ACADO_NX, ACADO_N + 1> predicted_states_;
    Eigen::Matrix<real_t, kCostStateSize, ACADO_N + 1> reference_states_; // +1 is the terminal node
    Eigen::Matrix<real_t, ACADO_NU, ACADO_N> reference_controls_;
    Eigen::Matrix<real_t, ACADO_NY, kWeightSize> w_;
    Eigen::Matrix<real_t, ACADO_NYN, ACADO_NYN> wn_;
    Eigen::Matrix<real_t, ACADO_NU, 1> current_control_;
    Eigen::Matrix<real_t, kCostStateSize, kCostStateSize> q_;
    Eigen::Matrix<real_t, ACADO_NU, ACADO_NU> r_;
    Eigen::Matrix<real_t, ACADO_NU, 1> control_lower_bound_;
    Eigen::Matrix<real_t, ACADO_NU, 1> control_upper_bound_;

    bool is_new_data_;
    int mpc_count_;

    MpcError mpc_error_;

public:
    MpcRosApplication(): nh_("~"), is_new_data_(true), mpc_count_(0), 
                        mpc_error_(MPC_OK){}
    virtual ~MpcRosApplication() 
    {
        mpcOff();
    }

    virtual void init() = 0;
    virtual void setReference() = 0;
    virtual void setParameter() {};
    virtual void setWeight() {};
    virtual void setOnlineData() {};
    virtual void setInputBounds() {};
    virtual void mpcProcess();
    virtual void mpcStart();
    virtual void mpcOff();
    virtual Eigen::Matrix<real_t, ACADO_NU, 1> getCurrentControl() 
    {
        return current_control_;
    }
    virtual MpcError getMpcError() 
    {
        return mpc_error_;
    }

    virtual Eigen::Matrix<real_t, ACADO_NU, 1> getControlLowerBound()
    {
        return control_lower_bound_;
    }    

    virtual Eigen::Matrix<real_t, ACADO_NU, 1> getControlUpperBound()
    {
        return control_upper_bound_;
    }         
};

class MpcFactory 
{
public:
    virtual ~MpcFactory() {}
    virtual MpcRosApplication* createMpcRosApplication() = 0;
};
