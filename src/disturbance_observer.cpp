#include <math.h>
#include <iostream>
#include "disturbance_observer.h"

DisturbanceObserver::DisturbanceObserver(double dt)
        : dt_(dt), l_(Eigen::VectorXd::Zero(8)), L_(Eigen::VectorXd::Zero(5)), z_(Eigen::VectorXd::Zero(10)),
			d_(Eigen::VectorXd::Zero(8)), dd_(Eigen::VectorXd::Zero(8)), ddd_(Eigen::VectorXd::Zero(8)),
			fl_(Eigen::VectorXd::Zero(4)), fn_(Eigen::VectorXd::Zero(5)) 
{
	// z_(1) = M_PI_2;
    // pole_ = -1.5;
    // l_ << 3*omega, 3*omega*omega, omega*omega*omega;
    // l_(0) = -pole_ * 2;
    // l_(1) = pow(l_(0),2) / 3;
    double omega = 1.5;
    l_.block<3, 1>(0, 0) << 3*omega, 3*omega*omega, omega*omega*omega;
    // l_.block<4, 1>(0, 0) << 4*omega, 6*omega*omega, 4*omega*omega*omega, omega*omega*omega*omega;

	// l_(0) = 5;
	// l_(1) = 6.25;

    // pole_ = -2.5;
    // // l_ << 3*omega, 3*omega*omega, omega*omega*omega;
    // l_(0) = -pole_ * 2;
    // l_(1) = pow(pole_,2);

	fl_(0) = 6;
	fl_(1) = 2;
	fl_(2) = 6;
	fl_(3) = 2;

	// L_ << 2, 1, 3, 3, 3;

	L_ << 5, 5, 5, 5, 5;

	fn_ << 0.3, 0.3, 0.3, 0.3, 0.3;

}
void DisturbanceObserver::update(const Eigen::VectorXd& y, double T, 
    double roll, double pitch, double yaw, double m, double g_z)
 {
    // 动力学模型
    double f_x = ( cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll) ) * T/m;
    double f_y = ( sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll) ) * T/m;
    double f_z = ( cos(pitch) * cos(roll) ) * T/m - g_z;

    // 误差计算
    Eigen::VectorXd e = y - z_.block<3, 1>(0, 0);

    // 状态更新
    z_(0) += (z_(3) + l_(0) * e(0)) * dt_;
    z_(1) += (z_(4) + l_(0) * e(1)) * dt_;
    z_(2) += (z_(5) + l_(0) * e(2)) * dt_;
    z_(3) += (f_x + d_(0) + l_(1) * e(0)) * dt_;
    z_(4) += (f_y + d_(1) + l_(1) * e(1)) * dt_;
    z_(5) += (f_z + d_(2) + l_(1) * e(2)) * dt_;

    // 扰动更新
    d_(0) += l_(2) * e(0) * dt_;
    d_(1) += l_(2) * e(1) * dt_;
    d_(2) += l_(2) * e(2) * dt_;
}

void DisturbanceObserver::update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, 
    double roll, double pitch, double yaw, double m, double g_z, const Eigen::MatrixXd& visual_jacob)
{
    // 动力学模型
    double f_x = ( cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll) ) * u(3)/m;
    double f_y = ( sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll) ) * u(3)/m;
    double f_z = ( cos(pitch) * cos(roll) ) * u(3)/m - g_z;

    // 误差计算
    Eigen::VectorXd e = y - z_.block<5, 1>(0, 0);

// 状态更新
    z_(0) += (visual_jacob(0,0)*z_(2) + visual_jacob(0,1)*z_(3) + visual_jacob(0,2)*z_(4)
            + visual_jacob(0,3)*u(0) + visual_jacob(0,4)*u(1) + visual_jacob(0,5)*u(2) + 
            d_(0) + l_(0) * e(0)) * dt_;
    z_(1) += (visual_jacob(1,0)*z_(2) + visual_jacob(1,1)*z_(3) + visual_jacob(1,2)*z_(4)
            + visual_jacob(1,3)*u(0) + visual_jacob(1,4)*u(1) + visual_jacob(1,5)*u(2) + 
            d_(1) + l_(0) * e(1)) * dt_;
    z_(2) += (f_x + d_(2) + l_(0) * e(2)) * dt_;
    z_(3) += (f_y + d_(3) + l_(0) * e(3)) * dt_;
    z_(4) += (f_z + d_(4) + l_(0) * e(4)) * dt_;

    // 扰动更新
    d_(0) += l_(1) * e(0) * dt_;
    d_(1) += l_(1) * e(1) * dt_;
    d_(2) += l_(1) * e(2) * dt_;
    d_(3) += l_(1) * e(3) * dt_;
    d_(4) += l_(1) * e(4) * dt_;
}

void DisturbanceObserver::finite_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, 
    double roll, double pitch, double yaw, double m, double g_z, const Eigen::MatrixXd& visual_jacob)
{
    // 动力学模型
    double f_x = ( cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll) ) * u(3)/m;
    double f_y = ( sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll) ) * u(3)/m;
    double f_z = ( cos(pitch) * cos(roll) ) * u(3)/m - g_z;

    // 误差计算
    Eigen::VectorXd e = y - z_.block<5, 1>(0, 0);
	double v0, v1, v2;

// 状态更新
    // z_(0) += (visual_jacob(0,0)*z_(2) + visual_jacob(0,1)*z_(3) + visual_jacob(0,2)*z_(4)
    //         + visual_jacob(0,3)*u(0) + visual_jacob(0,4)*u(1) + visual_jacob(0,5)*u(2) + 
    //         d_(0) + fl_(0) * std::copysign(std::pow(std::abs(e(0)), 0.5), e(0))) * dt_;
    // z_(1) += (visual_jacob(1,0)*z_(2) + visual_jacob(1,1)*z_(3) + visual_jacob(1,2)*z_(4)
    //         + visual_jacob(1,3)*u(0) + visual_jacob(1,4)*u(1) + visual_jacob(1,5)*u(2) + 
    //         d_(1) + fl_(0) * std::copysign(std::pow(std::abs(e(0)), 0.5), e(1))) * dt_;
    // z_(2) += (f_x + d_(2) + fl_(2) * std::copysign(std::pow(std::abs(e(2)), 0.5), e(2))) * dt_;
    // z_(3) += (f_y + d_(3) + fl_(2) * std::copysign(std::pow(std::abs(e(3)), 0.5), e(3))) * dt_;
    // z_(4) += (f_z + d_(4) + fl_(2) * std::copysign(std::pow(std::abs(e(4)), 0.5), e(4))) * dt_;

    // // 扰动更新
    // d_(0) += fl_(1) * std::copysign(std::pow(std::abs(e(0)), 0.5), e(0)) * dt_;
    // d_(1) += fl_(1) * std::copysign(std::pow(std::abs(e(1)), 0.5), e(1)) * dt_;
    // d_(2) += fl_(3) * std::copysign(std::pow(std::abs(e(2)), 0.5), e(2)) * dt_;
    // d_(3) += fl_(3) * std::copysign(std::pow(std::abs(e(3)), 0.5), e(3)) * dt_;
    // d_(4) += fl_(3) * std::copysign(std::pow(std::abs(e(4)), 0.5), e(4)) * dt_;

	v0 = d_(0) + 3 * std::pow(L_(0), 1/3) * std::copysign(std::pow(std::abs(e(0)), 2/3), e(0));
	z_(0) += (visual_jacob(0,0)*y(2) + visual_jacob(0,1)*y(3) + visual_jacob(0,2)*y(4)
            + visual_jacob(0,3)*u(0) + visual_jacob(0,4)*u(1) + visual_jacob(0,5)*u(2) + v0) * dt_;
	v1 = 1.5*std::pow(L_(0), 1/2) * std::copysign(std::pow(std::abs(v0-d_(0)), 1/2), v0-d_(0)) + dd_(0);		
	d_(0) += v1 * dt_;
	dd_(0) += 1.1*L_(0)*std::copysign(1.0, v1-dd_(0)) * dt_;

	v0 = d_(1) + 3 * std::pow(L_(1), 1/3) * std::copysign(std::pow(std::abs(e(1)), 2/3), e(1));
	z_(1) += (visual_jacob(1,0)*y(2) + visual_jacob(1,1)*y(3) + visual_jacob(1,2)*y(4)
            + visual_jacob(1,3)*u(0) + visual_jacob(1,4)*u(1) + visual_jacob(1,5)*u(2) + v0) * dt_;
	v1 = 1.5*std::pow(L_(1), 1/2) * std::copysign(std::pow(std::abs(v0-d_(1)), 1/2), v0-d_(1)) + dd_(1);		
	d_(1) += v1 * dt_;
	dd_(1) += 1.1*L_(1)*std::copysign(1.0, v1-dd_(1)) * dt_;

	v0 = d_(2) + 3 * std::pow(L_(2), 1/3) * std::copysign(std::pow(std::abs(e(2)), 2/3), e(2));
	z_(2) += (f_x + v0) * dt_;
	v1 = 1.5*std::pow(L_(2), 1/2) * std::copysign(std::pow(std::abs(v0-d_(2)), 1/2), v0-d_(2)) + dd_(2);		
	d_(2) += v1 * dt_;
	dd_(2) += 1.1*L_(2)*std::copysign(1.0, v1-dd_(2)) * dt_;

	v0 = d_(3) + 3 * std::pow(L_(3), 1/3) * std::copysign(std::pow(std::abs(e(3)), 2/3), e(3));
	z_(3) += (f_y + v0) * dt_;
	v1 = 1.5*std::pow(L_(3), 1/2) * std::copysign(std::pow(std::abs(v0-d_(3)), 1/2), v0-d_(3)) + dd_(3);		
	d_(3) += v1 * dt_;
	dd_(3) += 1.1*L_(3)*std::copysign(1.0, v1-dd_(3)) * dt_;

	v0 = d_(4) + 3 * std::pow(L_(4), 1/3) * std::copysign(std::pow(std::abs(e(4)), 2/3), e(4));
	z_(4) += (f_z + v0) * dt_;
	v1 = 1.5*std::pow(L_(4), 1/2) * std::copysign(std::pow(std::abs(v0-d_(4)), 1/2), v0-d_(4)) + dd_(4);		
	d_(4) += v1 * dt_;
	dd_(4) += 1.1*L_(4)*std::copysign(1.0, v1-dd_(4)) * dt_;

	// v0 = d_(0) + 8 * std::pow(L_(0), 1/4) * std::copysign(std::pow(std::abs(e(0)), 3/4), e(0));
	// z_(0) += (visual_jacob(0,0)*y(2) + visual_jacob(0,1)*y(3) + visual_jacob(0,2)*y(4)
    //         + visual_jacob(0,3)*u(0) + visual_jacob(0,4)*u(1) + visual_jacob(0,5)*u(2) + v0) * dt_;
	// v1 = 3*std::pow(L_(0), 1/3) * std::copysign(std::pow(std::abs(v0-d_(0)), 2/3), v0-d_(0)) + dd_(0);		
	// d_(0) += v1 * dt_;
	// v2 = 1.5*std::pow(L_(0), 1/2) * std::copysign(std::pow(std::abs(v1-dd_(0)), 1/2), v1-dd_(0)) + ddd_(0);
	// dd_(0) += v2 * dt_;
	// ddd_(0) += 1.1*L_(0)*std::copysign(1.0, v2-ddd_(0)) * dt_;

	// v0 = d_(1) + 8 * std::pow(L_(1), 1/4) * std::copysign(std::pow(std::abs(e(1)), 3/4), e(1));
	// z_(1) += (visual_jacob(1,0)*y(2) + visual_jacob(1,1)*y(3) + visual_jacob(1,2)*y(4)
	// 		+ visual_jacob(1,3)*u(0) + visual_jacob(1,4)*u(1) + visual_jacob(1,5)*u(2) + v0) * dt_;
	// v1 = 3*std::pow(L_(1), 1/3) * std::copysign(std::pow(std::abs(v0-d_(1)), 2/3), v0-d_(1)) + dd_(1);
	// d_(1) += v1 * dt_;
	// v2 = 1.5*std::pow(L_(1), 1/2) * std::copysign(std::pow(std::abs(v1-dd_(1)), 1/2), v1-dd_(1)) + ddd_(1);
	// dd_(1) += v2 * dt_;
	// ddd_(1) += 1.1*L_(1)*std::copysign(1.0, v2-ddd_(1)) * dt_;

	// v0 = d_(2) + 8 * std::pow(L_(2), 1/4) * std::copysign(std::pow(std::abs(e(2)), 3/4), e(2));
	// z_(2) += (f_x + v0) * dt_;
	// v1 = 3*std::pow(L_(2), 1/3) * std::copysign(std::pow(std::abs(v0-d_(2)), 2/3), v0-d_(2)) + dd_(2);
	// d_(2) += v1 * dt_;
	// v2 = 1.5*std::pow(L_(2), 1/2) * std::copysign(std::pow(std::abs(v1-dd_(2)), 1/2), v1-dd_(2)) + ddd_(2);
	// dd_(2) += v2 * dt_;
	// ddd_(2) += 1.1*L_(2)*std::copysign(1.0, v2-ddd_(2)) * dt_;

	// v0 = d_(3) + 8 * std::pow(L_(3), 1/4) * std::copysign(std::pow(std::abs(e(3)), 3/4), e(3));
	// z_(3) += (f_y + v0) * dt_;
	// v1 = 3*std::pow(L_(3), 1/3) * std::copysign(std::pow(std::abs(v0-d_(3)), 2/3), v0-d_(3)) + dd_(3);
	// d_(3) += v1 * dt_;
	// v2 = 1.5*std::pow(L_(3), 1/2) * std::copysign(std::pow(std::abs(v1-dd_(3)), 1/2), v1-dd_(3)) + ddd_(3);
	// dd_(3) += v2 * dt_;
	// ddd_(3) += 1.1*L_(3)*std::copysign(1.0, v2-ddd_(3)) * dt_;

	// v0 = d_(4) + 8 * std::pow(L_(4), 1/4) * std::copysign(std::pow(std::abs(e(4)), 3/4), e(4));
	// z_(4) += (f_z + v0) * dt_;
	// v1 = 3*std::pow(L_(4), 1/3) * std::copysign(std::pow(std::abs(v0-d_(4)), 2/3), v0-d_(4)) + dd_(4);
	// d_(4) += v1 * dt_;
	// v2 = 1.5*std::pow(L_(4), 1/2) * std::copysign(std::pow(std::abs(v1-dd_(4)), 1/2), v1-dd_(4)) + ddd_(4);
	// dd_(4) += v2 * dt_;
	// ddd_(4) += 1.1*L_(4)*std::copysign(1.0, v2-ddd_(4)) * dt_;
}

void DisturbanceObserver::nonlinear_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
        double roll, double pitch, double yaw, double m, double g_z, const Eigen::MatrixXd& visual_jacob)
{
	// 动力学模型
    double f_x = ( cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll) ) * u(3)/m;
    double f_y = ( sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll) ) * u(3)/m;
    double f_z = ( cos(pitch) * cos(roll) ) * u(3)/m - g_z;

    // 误差计算
    Eigen::VectorXd e = y - z_.block<5, 1>(0, 0);	

	z_(0) += (-fn_(0)*z_(0) - fn_(0)*(fn_(0)*y(0)+visual_jacob(0,0)*y(2) + visual_jacob(0,1)*y(3) + visual_jacob(0,2)*y(4)
            + visual_jacob(0,3)*u(0) + visual_jacob(0,4)*u(1) + visual_jacob(0,5)*u(2))) * dt_;
	d_(0) = z_(0) + fn_(0)*y(0);

	z_(1) += (-fn_(1)*z_(1) - fn_(1)*(fn_(1)*y(1)+visual_jacob(1,0)*y(2) + visual_jacob(1,1)*y(3) + visual_jacob(1,2)*y(4)
            + visual_jacob(1,3)*u(0) + visual_jacob(1,4)*u(1) + visual_jacob(1,5)*u(2))) * dt_;
	d_(1) = z_(1) + fn_(1)*y(1);

	z_(2) += (-fn_(2)*z_(2) - fn_(2)*(fn_(2)*y(2)+f_x)) * dt_;
	d_(2) = z_(2) + fn_(2)*y(2);

	z_(3) += (-fn_(3)*z_(3) - fn_(3)*(fn_(3)*y(3)+f_y)) * dt_;
	d_(3) = z_(3) + fn_(3)*y(3);

	z_(4) += (-fn_(4)*z_(4) - fn_(4)*(fn_(4)*y(4)+f_z)) * dt_;
	d_(4) = z_(4) + fn_(4)*y(4);
}

void DisturbanceObserver::kalman_geso_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, double m, double g_z)
{
    Eigen::MatrixXd A(kf_.state_size_, kf_.state_size_);
    A.setZero();

    A.block<3,3>(0,7) = Eigen::Matrix3d::Identity();

    // Unpack quaternion from y
    double qw = y(3), qx = y(4), qy = y(5), qz = y(6);

    // Bu matrix (control input T, w_x, w_y, w_z)
    Eigen::MatrixXd B_u(kf_.state_size_, kf_.control_size_);
    Eigen::MatrixXd B_d(kf_.state_size_, 3);
    B_u.setZero();

    B_u(3,0) = -0.5 * qx;  // ∂q_w/∂w_x
    B_u(3,1) = -0.5 * qy;  // ∂q_w/∂w_y
    B_u(3,2) = -0.5 * qz;  // ∂q_w/∂w_z

    B_u(4,0) =  0.5 * qw;  // ∂q_x/∂w_x
    B_u(4,1) = -0.5 * qz;  // ∂q_x/∂w_y
    B_u(4,2) =  0.5 * qy;  // ∂q_x/∂w_z

    B_u(5,0) =  0.5 * qz;  // ∂q_y/∂w_x
    B_u(5,1) =  0.5 * qw;  // ∂q_y/∂w_y
    B_u(5,2) = -0.5 * qx;  // ∂q_y/∂w_z

    B_u(6,0) = -0.5 * qy;  // ∂q_z/∂w_x
    B_u(6,1) =  0.5 * qx;  // ∂q_z/∂w_y
    B_u(6,2) =  0.5 * qw;  // ∂q_z/∂w_z

    // Simplified force direction from quaternion: here using T * gravity-aligned vector
    Eigen::Vector3d z_b;
    z_b << 2 * (qw*qy + qx*qz),
    2 * (qy*qz - qw*qx),
    1 - 2 * (qx*qx + qy*qy);
    B_u.block<3,1>(7, 3) = z_b / m;  // force input from thrust

    B_d.block<3,3>(7, 0) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd A2 = A * A;
    Eigen::MatrixXd A3 = A2 * A;
    Eigen::MatrixXd A4 = A3 * A;

    // ESKF Discretization
    Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(kf_.state_size_, kf_.state_size_) 
                            + dt_ * A
                            + 0.5 * dt_ * dt_ * A2
                            + (1.0/6.0) * dt_ * dt_ * dt_ * A3
                            + (1.0/24.0) * dt_ * dt_ * dt_ * dt_ * A4;
    A = temp;
    B_u = dt_ * B_u;
    B_d = dt_ * B_d;

    // gravity on v_z and disturbance estimation saturation
    Eigen::VectorXd tail = B_d * d_.block<3, 1>(0, 0) 
        - g_z * dt_ * Eigen::VectorXd::Unit(kf_.state_size_, 9);  
   
	kf_.predict(A, B_u, tail, u);
	kf_.update(y.block<7, 1>(0, 0));
	// 状态更新

	Eigen::VectorXd estimated_states = kf_.getState();
	    // 动力学模型
    double f_x = z_b(0) * u(3)/m;
    double f_y = z_b(1) * u(3)/m;
    double f_z = z_b(2) * u(3)/m - g_z;

    // 误差计算
    Eigen::VectorXd e = y.block<3, 1>(0, 0) - z_.block<3, 1>(0, 0);
	// e(2) = estimated_states(0) - z_(2);
	// e(3) = estimated_states(1) - z_(3);
	// e(4) = estimated_states(2) - z_(4);

// 状态更新
    z_(0) += (z_(3) + l_(0) * e(0)) * dt_;
    z_(1) += (z_(4) + l_(0) * e(1)) * dt_;
    z_(2) += (z_(5) + l_(0) * e(2)) * dt_;
    z_(3) += (f_x + d_(0) + l_(1) * e(0)) * dt_;
    z_(4) += (f_y + d_(1) + l_(1) * e(1)) * dt_;
    z_(5) += (f_z + d_(2) + l_(1) * e(2)) * dt_;

    // 扰动更新
    d_(0) += l_(2) * e(0) * dt_;
    d_(1) += l_(2) * e(1) * dt_;
    d_(2) += l_(2) * e(2) * dt_;
}

void DisturbanceObserver::kalman_gpio_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, double m, double g_z)
{
    Eigen::MatrixXd A(kf_.state_size_, kf_.state_size_);
    A.setZero();

    A.block<3,3>(0,7) = Eigen::Matrix3d::Identity();

    // Unpack quaternion from y
    double qw = y(3), qx = y(4), qy = y(5), qz = y(6);

    // Bu matrix (control input T, w_x, w_y, w_z)
    Eigen::MatrixXd B_u(kf_.state_size_, kf_.control_size_);
    Eigen::MatrixXd B_d(kf_.state_size_, 3);
    B_u.setZero();

    B_u(3,0) = -0.5 * qx;  // ∂q_w/∂w_x
    B_u(3,1) = -0.5 * qy;  // ∂q_w/∂w_y
    B_u(3,2) = -0.5 * qz;  // ∂q_w/∂w_z

    B_u(4,0) =  0.5 * qw;  // ∂q_x/∂w_x
    B_u(4,1) = -0.5 * qz;  // ∂q_x/∂w_y
    B_u(4,2) =  0.5 * qy;  // ∂q_x/∂w_z

    B_u(5,0) =  0.5 * qz;  // ∂q_y/∂w_x
    B_u(5,1) =  0.5 * qw;  // ∂q_y/∂w_y
    B_u(5,2) = -0.5 * qx;  // ∂q_y/∂w_z

    B_u(6,0) = -0.5 * qy;  // ∂q_z/∂w_x
    B_u(6,1) =  0.5 * qx;  // ∂q_z/∂w_y
    B_u(6,2) =  0.5 * qw;  // ∂q_z/∂w_z

    // Simplified force direction from quaternion: here using T * gravity-aligned vector
    Eigen::Vector3d z_b;
    z_b << 2 * (qw*qy + qx*qz),
    2 * (qy*qz - qw*qx),
    1 - 2 * (qx*qx + qy*qy);
    B_u.block<3,1>(7, 3) = z_b / m;  // force input from thrust

    B_d.block<3,3>(7, 0) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd A2 = A * A;
    Eigen::MatrixXd A3 = A2 * A;
    Eigen::MatrixXd A4 = A3 * A;

    // ESKF Discretization
    Eigen::MatrixXd temp = Eigen::MatrixXd::Identity(kf_.state_size_, kf_.state_size_) 
                            + dt_ * A
                            + 0.5 * dt_ * dt_ * A2
                            + (1.0/6.0) * dt_ * dt_ * dt_ * A3
                            + (1.0/24.0) * dt_ * dt_ * dt_ * dt_ * A4;
    A = temp;
    B_u = dt_ * B_u;
    B_d = dt_ * B_d;

    // gravity on v_z and disturbance estimation saturation
    Eigen::VectorXd tail = B_d * d_.block<3, 1>(0, 0) 
        - g_z * dt_ * Eigen::VectorXd::Unit(kf_.state_size_, 9);  
   
	kf_.predict(A, B_u, tail, u);
	kf_.update(y.block<7, 1>(0, 0));
	// 状态更新

	Eigen::VectorXd estimated_states = kf_.getState();
	    // 动力学模型
    double f_x = z_b(0) * u(3)/m;
    double f_y = z_b(1) * u(3)/m;
    double f_z = z_b(2) * u(3)/m - g_z;

    // 误差计算
    Eigen::VectorXd e = y.block<3, 1>(0, 0) - z_.block<3, 1>(0, 0);
	// e(2) = estimated_states(0) - z_(2);
	// e(3) = estimated_states(1) - z_(3);
	// e(4) = estimated_states(2) - z_(4);

// 状态更新
    z_(0) += (z_(3) + l_(0) * e(0)) * dt_;
    z_(1) += (z_(4) + l_(0) * e(1)) * dt_;
    z_(2) += (z_(5) + l_(0) * e(2)) * dt_;
    z_(3) += (f_x + d_(0) + l_(1) * e(0)) * dt_;
    z_(4) += (f_y + d_(1) + l_(1) * e(1)) * dt_;
    z_(5) += (f_z + d_(2) + l_(1) * e(2)) * dt_;

    // 扰动更新
    d_(0) += (dd_(0) + l_(2) * e(0)) * dt_;
    d_(1) += (dd_(1) + l_(2) * e(1)) * dt_;
    d_(2) += (dd_(2) + l_(2) * e(2)) * dt_;
    // 扰动一阶导
    dd_(0) += l_(3) * e(0) * dt_;
    dd_(1) += l_(3) * e(1) * dt_;
    dd_(2) += l_(3) * e(2) * dt_;
}

void DisturbanceObserver::kalman_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
        double roll, double pitch, double yaw, double m, double g_z, const Eigen::MatrixXd& visual_jacob)
{
    Eigen::MatrixXd A(kf_.state_size_, kf_.state_size_);
    A.setZero();
    A.block<2, 3>(0,2) = visual_jacob.leftCols(3);
    A.block<5,5>(0,5) = Eigen::MatrixXd::Identity(5, 5);
    // 定义矩阵 G
    Eigen::VectorXd G(kf_.state_size_);
    G.setZero();
    G(4) = 1;

	// R_WB第三列
	Eigen::Vector3d R_WB_col2;
	R_WB_col2 << cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll),
				 sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll),
				 cos(pitch) * cos(roll);


    // 定义矩阵 B_u
    Eigen::MatrixXd B_u(kf_.state_size_, kf_.control_size_);
    B_u.setZero();
    B_u.block<2,3>(0,0) = visual_jacob.rightCols(3);
    B_u.block<3,1>(2, 3) = R_WB_col2 / m;


    A = Eigen::MatrixXd::Identity(kf_.state_size_, kf_.state_size_) + dt_ * A;
	B_u = dt_ * B_u;
	G = dt_ * G;

	Eigen::VectorXd tail =  - g_z * G;

	kf_.predict(A, B_u, tail, u);
	kf_.update(y.block<5, 1>(0, 0));

    z_.block<5, 1>(0, 0) = kf_.getState().block<5, 1>(0, 0);
    d_.block<5, 1>(0, 0) = kf_.getState().block<5, 1>(5, 0);
}

void DisturbanceObserver::eskf_update(
    const Eigen::VectorXd& y,              // measurement: dim = 10
    const Eigen::VectorXd& u,              // control input: dim = 4
    double m, double g_z)
{
    Eigen::MatrixXd A(eskf_.state_size_ + eskf_.extended_size_, 
                        eskf_.state_size_ + eskf_.extended_size_);
    A.setZero();

    A.block<3,3>(0,7) = Eigen::Matrix3d::Identity();

    // Partial of velocity w.r.t disturbances
    A.block<3,3>(7,10) = Eigen::Matrix3d::Identity();
    
    // Unpack quaternion from y
    double qw = y(3), qx = y(4), qy = y(5), qz = y(6);

    // Bu matrix (control input T, w_x, w_y, w_z)
    Eigen::MatrixXd B_u(eskf_.state_size_ + eskf_.extended_size_, eskf_.control_size_);
    B_u.setZero();

    B_u(3,0) = -0.5 * qx;  // ∂q_w/∂w_x
    B_u(3,1) = -0.5 * qy;  // ∂q_w/∂w_y
    B_u(3,2) = -0.5 * qz;  // ∂q_w/∂w_z

    B_u(4,0) =  0.5 * qw;  // ∂q_x/∂w_x
    B_u(4,1) = -0.5 * qz;  // ∂q_x/∂w_y
    B_u(4,2) =  0.5 * qy;  // ∂q_x/∂w_z

    B_u(5,0) =  0.5 * qz;  // ∂q_y/∂w_x
    B_u(5,1) =  0.5 * qw;  // ∂q_y/∂w_y
    B_u(5,2) = -0.5 * qx;  // ∂q_y/∂w_z

    B_u(6,0) = -0.5 * qy;  // ∂q_z/∂w_x
    B_u(6,1) =  0.5 * qx;  // ∂q_z/∂w_y
    B_u(6,2) =  0.5 * qw;  // ∂q_z/∂w_z

    // Simplified force direction from quaternion: here using T * gravity-aligned vector
    Eigen::Vector3d z_b;
    z_b << 2 * (qw*qy + qx*qz),
        2 * (qy*qz - qw*qx),
        1 - 2 * (qx*qx + qy*qy);
    B_u.block<3,1>(7, 3) = z_b / m;  // force input from thrust

    // ESKF Discretization
    A = Eigen::MatrixXd::Identity(eskf_.state_size_ + eskf_.extended_size_, eskf_.state_size_ + eskf_.extended_size_) + dt_ * A;
    B_u = dt_ * B_u;

    Eigen::VectorXd tail = - g_z * dt_ * Eigen::VectorXd::Unit(eskf_.state_size_ + eskf_.extended_size_, 9);  // gravity on v_z

    eskf_.set_Ak(A);
    eskf_.predict(Eigen::VectorXd::Zero(eskf_.extended_size_), tail, B_u, u);
    eskf_.update(y);  // measurement: full 10D state (p, q, v)

    z_.head<10>() = eskf_.get_X_hat();
    d_.head<3>() = eskf_.get_F_hat();
}


void DisturbanceObserver::update(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
                 double m, double g_z, const Eigen::MatrixXd& visual_jacob)
{
    double f_x = ( cos(z_(7)) * sin(z_(6)) * cos(z_(5)) + sin(z_(7)) * sin(z_(5)) ) * u(3)/m;
    double f_y = ( sin(z_(7)) * sin(z_(6)) * cos(z_(5)) - cos(z_(7)) * sin(z_(5)) ) * u(3)/m;
    double f_z = ( cos(z_(6)) * cos(z_(5)) ) * u(3)/m - g_z;

    // 误差计算
    Eigen::VectorXd e = y - z_.block<8, 1>(0, 0);

    // 状态更新
    z_(0) += (visual_jacob(0,0)*z_(2) + visual_jacob(0,1)*z_(3) + visual_jacob(0,2)*z_(4)
            + visual_jacob(0,3)*u(0) + visual_jacob(0,4)*u(1) + visual_jacob(0,5)*u(2) + 
            d_(0) + l_(0) * e(0)) * dt_;
    z_(1) += (visual_jacob(1,0)*z_(2) + visual_jacob(1,1)*z_(3) + visual_jacob(1,2)*z_(4)
            + visual_jacob(1,3)*u(0) + visual_jacob(1,4)*u(1) + visual_jacob(1,5)*u(2) + 
            d_(1) + l_(0) * e(1)) * dt_;
    z_(2) += (f_x + d_(2) + l_(0) * e(2)) * dt_;
    z_(3) += (f_y + d_(3) + l_(0) * e(3)) * dt_;
    z_(4) += (f_z + d_(4) + l_(0) * e(4)) * dt_;
    z_(5) += (u(0) + sin(z_(5)) * tan(z_(6)) * u(1) + 
            cos(z_(5)) * tan(z_(6)) * u(2) + d_(5) + l_(0) * e(5)) * dt_;
    z_(6) += (u(1) * cos(z_(5)) - u(2) * sin(z_(5)) + d_(6) + l_(0) * e(6)) * dt_;
    z_(7) += (sin(z_(5)) / cos(z_(6)) * u(1) + 
            cos(z_(5)) / cos(z_(6)) * u(2) + d_(7) + l_(0) * e(7)) * dt_;
    // 扰动更新
    d_(0) += l_(1) * e(0) * dt_;
    d_(1) += l_(1) * e(1) * dt_;
    d_(2) += l_(1) * e(2) * dt_;
    d_(3) += l_(1) * e(3) * dt_;
    d_(4) += l_(1) * e(4) * dt_;
    d_(5) += l_(1) * e(5) * dt_;
    d_(6) += l_(1) * e(6) * dt_;
    d_(7) += l_(1) * e(7) * dt_;
}