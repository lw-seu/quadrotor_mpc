#pragma once

#include <Eigen/Dense>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

static constexpr int kCostStateSize = ACADO_NY - ACADO_NU;
static constexpr int kWeightSize = ACADO_NY * ACADO_N;

// enum MpcErrors{
//     MPC_OK = 0,
//     DATA_LOST,
// };

// MpcErrors mpc_error = MPC_OK;

class MpcWrapper
{
private:
    Eigen::Map<Eigen::Matrix<real_t, ACADO_NX, 1, Eigen::ColMajor>>
        acado_current_states_{acadoVariables.x0};

    Eigen::Map<Eigen::Matrix<real_t, ACADO_NX, ACADO_N + 1, Eigen::ColMajor>>
        acado_predicted_states_{acadoVariables.x};

    Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, ACADO_N, Eigen::ColMajor>>
        acado_controls_{acadoVariables.u};

    Eigen::Map<Eigen::Matrix<real_t, ACADO_NY, ACADO_N, Eigen::ColMajor>>
        acado_reference_{acadoVariables.y};

    Eigen::Map<Eigen::Matrix<real_t, ACADO_NYN, 1, Eigen::ColMajor>>
        acado_terminal_reference_{acadoVariables.yN};

    Eigen::Map<Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1, Eigen::ColMajor>>
        acado_online_data_{acadoVariables.od};

    Eigen::Map<Eigen::Matrix<real_t, ACADO_NY, kWeightSize, Eigen::ColMajor>>
        acado_weight_{acadoVariables.W};
    Eigen::Map<Eigen::Matrix<real_t, ACADO_NYN, ACADO_NYN, Eigen::ColMajor>>
        acado_terminal_weight_{acadoVariables.WN};
    Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, ACADO_N, Eigen::ColMajor>>
        acado_lower_bounds_{acadoVariables.lbValues};
    Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, ACADO_N, Eigen::ColMajor>>
        acado_upper_bounds_{acadoVariables.ubValues};
    // Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, ACADO_N, Eigen::ColMajor>>
    //     acado_lower_affine_bounds_{acadoVariables.lbAValues};
    // Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, ACADO_N, Eigen::ColMajor>>
    //     acado_upper_affine_bounds_{acadoVariables.ubAValues};
    

public:
    MpcWrapper();

    void initMpc(
        const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NX, 1>> &current_states,
        const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NX, ACADO_N + 1>> &predicted_states,
        const Eigen::Ref<const Eigen::Matrix<real_t, kCostStateSize, ACADO_N + 1>> &reference_states,
        const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, ACADO_N>> &reference_inputs,
        const Eigen::Ref<const Eigen::Matrix<real_t, kCostStateSize, kCostStateSize>>& Q,
        const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, ACADO_NU>>& R);
    bool setMpcReference(
        const Eigen::Ref<const Eigen::Matrix<real_t, kCostStateSize, ACADO_N + 1>>& ref_states,
        const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, ACADO_N>>& ref_control);
    bool setMpcWeight(
        const Eigen::Ref<const Eigen::Matrix<real_t, kCostStateSize, kCostStateSize>>& Q,
        const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, ACADO_NU>>& R,
        real_t state_cost_scaling = 0.0, real_t input_cost_scaling = 0.0);
    bool setMpcWeight(int index, 
        const Eigen::Ref<const Eigen::Matrix<real_t, kCostStateSize, kCostStateSize>>& Q, 
        const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, ACADO_NU>>& R); // 单独修改窗口中某一步状态的权重矩阵
    bool setMpcOnlineData(
        const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NOD, ACADO_N + 1>>& online_data);
    // bool setMpcAffineBounds(
    //     const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, 1>>& lower_affine_bounds,
    //     const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NU, 1>>& upper_affine_bounds);
    bool solveMpc(const Eigen::Ref<const Eigen::Matrix<real_t, ACADO_NX, 1>>& current_states);
    void getMpcControls(Eigen::Ref<Eigen::Matrix<real_t, ACADO_NU, ACADO_N>> controls);
    void getMpcControl(Eigen::Ref<Eigen::Matrix<real_t, ACADO_NU, 1>> control, int index);
    void getMpcBound(Eigen::Ref<Eigen::Matrix<real_t, ACADO_NU, 1>> lower_bound,
                      Eigen::Ref<Eigen::Matrix<real_t, ACADO_NU, 1>> upper_bound);
    void getMpcWeight(
        Eigen::Ref<Eigen::Matrix<real_t, kCostStateSize, kCostStateSize>> Q,
        Eigen::Ref<Eigen::Matrix<real_t, ACADO_NU, ACADO_NU>> R);
    void getMpcWeight(int index,
        Eigen::Ref<Eigen::Matrix<real_t, kCostStateSize, kCostStateSize>> Q,
        Eigen::Ref<Eigen::Matrix<real_t, ACADO_NU, ACADO_NU>> R);
};
