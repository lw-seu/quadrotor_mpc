#pragma once

#include <Eigen/Dense>
#include <deque>

class DiscreteKalmanFilter {
public:
    DiscreteKalmanFilter(int state_size, int measurement_size, int control_size) {
        // 初始化状态向量、协方差矩阵等
        x_ = Eigen::VectorXd(state_size);  // 状态向量
        P_ = Eigen::MatrixXd(state_size, state_size);  // 状态协方差矩阵
        A_ = Eigen::MatrixXd(state_size, state_size);  // 系统矩阵A
        B_ = Eigen::MatrixXd(state_size, control_size);  // 控制矩阵B
        H_ = Eigen::MatrixXd(measurement_size, state_size);  // 测量矩阵H
        R_ = Eigen::MatrixXd(measurement_size, measurement_size);  // 测量噪声协方差矩阵R
        Q_ = Eigen::MatrixXd(state_size, state_size);  // 过程噪声协方差矩阵Q
        Qa_ = Eigen::MatrixXd(state_size, state_size);  // 自适应噪声协方差矩阵Qa
        Ck_ = Eigen::MatrixXd(measurement_size, measurement_size);  // 自适应噪声协方差矩阵Ck_
        I_ = Eigen::MatrixXd::Identity(state_size, state_size);  // 单位矩阵
        K_ = Eigen::MatrixXd(state_size, measurement_size); // 卡尔曼增益
        innovation_ = Eigen::VectorXd(measurement_size);  // 新息（测量残差）
        state_size_ = state_size;
        measurement_size_ = measurement_size;
        control_size_ = control_size;
    }

    void init(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance,
              const Eigen::MatrixXd& system_matrix, const Eigen::MatrixXd& control_matrix, 
              const Eigen::MatrixXd& measurement_matrix, const Eigen::MatrixXd& measurement_noise, 
              const Eigen::MatrixXd& process_noise) {
        x_ = initial_state;  // 初始状态
        P_ = initial_covariance;  // 初始协方差矩阵
        A_ = system_matrix;  // 系统矩阵A
        B_ = control_matrix;  // 控制矩阵B
        H_ = measurement_matrix;  // 测量矩阵H
        R_ = measurement_noise;  // 测量噪声协方差矩阵R
        Q_ = process_noise;  // 过程噪声协方差矩阵Q
    }

    void init(const Eigen::VectorXd& initial_state)
    {
        innovation_ = Eigen::VectorXd::Zero(measurement_size_);
        m_ = 50;
        x_ = initial_state;
        P_ = 0.01 * Eigen::MatrixXd::Identity(state_size_, state_size_);
        P_.block<3,3>(7,7) = 0.1*Eigen::MatrixXd::Identity(3, 3);
        Q_ = 0.001 * Eigen::MatrixXd::Identity(state_size_, state_size_);
        Q_.block<3,3>(7,7) = 0.0016*Eigen::MatrixXd::Identity(3, 3);
        Qa_ = Q_;
        R_ = 0.0001 * Eigen::MatrixXd::Identity(measurement_size_, measurement_size_);
        // R_(0,0) = 0.01;
        // R_(1,1) = 0.01;
        H_ = Eigen::MatrixXd::Zero(measurement_size_, state_size_);
        H_.block(0, 0, measurement_size_, measurement_size_) = Eigen::MatrixXd::Identity(measurement_size_, measurement_size_);
        A_ = Eigen::MatrixXd::Identity(state_size_, state_size_);
        B_ = Eigen::MatrixXd::Zero(state_size_, control_size_);
        Ck_ = Eigen::MatrixXd::Zero(measurement_size_, measurement_size_);
    }
    void predict(const Eigen::MatrixXd A, const Eigen::MatrixXd Bu, const Eigen::VectorXd tail, const Eigen::VectorXd& control_input) {
        // 预测步骤：x = A * x + B * u
        B_ = Bu;
        A_ = A;
        x_ = A_ * x_ + B_ * control_input + tail;
        
        // 更新协方差矩阵
        // P_ = A_ * P_ * A_.transpose() + Q_;
        P_ = A_ * P_ * A_.transpose() + Qa_;
    }

    void update(const Eigen::VectorXd& measurement) {
        // 更新步骤
        // 计算创新（测量残差）
        innovation_ = measurement - H_ * x_;

        samples_.push_back(innovation_);
        Ck_ += innovation_ * innovation_.transpose();
        if (samples_.size() > m_) {
            Eigen::VectorXd old_q = samples_.front();
            Ck_ -= old_q * old_q.transpose();
            samples_.pop_front();
        }

        // R_ = Ck_/static_cast<double>(samples_.size()) - H_ * P_ * H_.transpose();
        // std::cout << "R_: " << R_ << std::endl;

        // 计算卡尔曼增益
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        K_ = P_ * H_.transpose() * S.inverse();

        // 更新状态估计
        x_ = x_ + K_ * innovation_;

        // 更新协方差矩阵
        P_ = (I_ - K_ * H_) * P_;
        // std::cout << "P" << P_ << std::endl;

        
        Qa_ = K_ * Ck_/static_cast<double>(samples_.size()) * K_.transpose();
        // std::cout << "Qa_: " << Qa_ << std::endl;
    }

    Eigen::VectorXd getState() const {
        return x_;
    }

    Eigen::MatrixXd getK() const {
        return K_;
    }

    Eigen::MatrixXd get_cov() {
        return P_;
    }

    Eigen::MatrixXd getA() {
        return A_;
    }

    Eigen::MatrixXd getCk() {
        return Ck_/static_cast<double>(samples_.size());
    }

    int state_size_;
    int measurement_size_;
    int control_size_;
    Eigen::MatrixXd Qa_; // 自适应噪声协方差矩阵

private:
    Eigen::VectorXd x_;  // 状态向量
    Eigen::VectorXd innovation_;  // 新息（测量残差）
    Eigen::MatrixXd P_;  // 状态协方差矩阵
    Eigen::MatrixXd A_;  // 系统矩阵A
    Eigen::MatrixXd B_;  // 控制矩阵B
    Eigen::MatrixXd H_;  // 测量矩阵H
    Eigen::MatrixXd R_;  // 测量噪声协方差矩阵R
    Eigen::MatrixXd Q_;  // 过程噪声协方差矩阵Q
    Eigen::MatrixXd I_;  // 单位矩阵
    Eigen::MatrixXd K_; // 卡尔曼增益
    Eigen::MatrixXd Ck_; // 新息协方差矩阵
    std::deque<Eigen::VectorXd> samples_;
    int m_;
};

class ESKF {
public:
    // **构造函数**
    ESKF(int state_dim, int extended_dim, int control_dim, int measurement_dim) {
        int total_dim = state_dim + extended_dim;

        m_ = 300; // innovation covariance estimation window size

        // **状态初始化**
        X_hat_ = Eigen::VectorXd::Zero(state_dim);
        F_hat_ = Eigen::VectorXd::Zero(extended_dim);
        G_hat_ = Eigen::VectorXd::Zero(extended_dim);
        innovation_ = Eigen::VectorXd::Zero(measurement_dim);
        qk_ = Eigen::VectorXd::Ones(extended_dim) * 1e-4; // 默认 qk_ 设为 0.1，可调
        // qk_ = Eigen::VectorXd::Zero(extended_dim); 

        // **误差协方差矩阵初始化**
        P_ = Eigen::MatrixXd::Identity(total_dim, total_dim) * 0.1;
        // P_.block(7, 7, 3, 3) = 0.01 * Eigen::MatrixXd::Identity(3, 3);
        P_.block(state_dim, state_dim, extended_dim, extended_dim) = 1 * Eigen::MatrixXd::Identity(extended_dim, extended_dim);

        // **系统矩阵初始化**
        A_ = Eigen::MatrixXd::Identity(total_dim, total_dim); // A_k 需由外部提供
        B_ = Eigen::MatrixXd::Zero(total_dim, extended_dim);
        B_.block(state_dim, 0, extended_dim, extended_dim) = Eigen::MatrixXd::Identity(extended_dim, extended_dim);
        Bu_= Eigen::MatrixXd::Zero(total_dim, control_dim);
        C_ = Eigen::MatrixXd::Zero(measurement_dim, total_dim);
        C_.block(0, 0, measurement_dim, state_dim) = Eigen::MatrixXd::Identity(state_dim, state_dim);

        // **噪声协方差初始化**
        Qk_ = Eigen::MatrixXd::Zero(extended_dim, extended_dim);
        Q1_ = Eigen::MatrixXd::Zero(total_dim, total_dim);
        Q2_ = Eigen::MatrixXd::Zero(total_dim, total_dim);
        Q2_.block(0, 0, state_dim, state_dim) = Eigen::MatrixXd::Identity(state_dim, state_dim) * 10;
        // Q2_.block(7, 7, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * 0.01;
        Qk_ = extended_dim * qk_.asDiagonal();
        Q1_.block(state_dim, state_dim, extended_dim, extended_dim) = 4 * Qk_;
        R_ = Eigen::MatrixXd::Identity(state_dim, state_dim) * 0.0001;
        Ck_ = Eigen::MatrixXd::Zero(measurement_dim, measurement_dim);
        Qa_ = Eigen::MatrixXd::Zero(total_dim, total_dim);

        // **计算初始 θ**
        compute_theta();

        std::cout << "theta: " << theta_ << std::endl;

        state_size_ = state_dim;
        measurement_size_ = measurement_dim;
        control_size_ = control_dim;
        extended_size_ = extended_dim;
    }

    void init(const Eigen::VectorXd& initial_state)
    {
        X_hat_ = initial_state;
    }
    Eigen::VectorXd get_X_hat() {
        return X_hat_;
    }

    Eigen::MatrixXd get_F_hat() {
        return F_hat_;
    }

    Eigen::MatrixXd get_cov() {
        return P_;
    }

    // **外部提供 A_k**
    void set_Ak(const Eigen::MatrixXd& Ak) {
        A_ = Ak;
    }

    void set_MPC_Ak(double obs_dt, double mpc_dt) {
        A_ = (A_ - Eigen::MatrixXd::Identity(state_size_ + extended_size_, state_size_ + extended_size_))/obs_dt * mpc_dt 
                + Eigen::MatrixXd::Identity(state_size_ + extended_size_, state_size_ + extended_size_);
    }

    // **外部设定 qk_**
    void set_qk(const Eigen::VectorXd& new_qk) {
        if (new_qk.size() == qk_.size()) {
            qk_ = new_qk;
        } else {
            std::cerr << "Error: qk size mismatch!" << std::endl;
        }
    }

    // **计算 θ**
    void compute_theta() {
        theta_ = std::sqrt(Q1_.trace() / P_.trace());
    }

    // **计算 Ĝ（饱和函数）**
    Eigen::VectorXd compute_G_hat(const Eigen::VectorXd& G_bar) {
        Eigen::VectorXd G_hat(G_bar.size());
        for (int i = 0; i < G_bar.size(); ++i) {
            double bound = std::sqrt(qk_(i));  // 直接使用已知 qk_
            G_hat(i) = std::max(std::min(G_bar(i), bound), -bound);
        }
        return G_hat;
    }

    // **计算卡尔曼增益 K_k**
    Eigen::MatrixXd compute_Kk() {
        Eigen::MatrixXd temp = C_ * P_ * C_.transpose() + (1.0 / (1 + theta_)) * R_;
        return -A_ * P_ * C_.transpose() * temp.inverse();
    }

    Eigen::MatrixXd compute_Kk(const Eigen::MatrixXd& P) {
        Eigen::MatrixXd temp = C_ * P * C_.transpose() + (1.0 / (1 + theta_)) * R_;
        return -A_ * P * C_.transpose() * temp.inverse();
    }
    

    // **预测更新**
    void predict(const Eigen::VectorXd& G_bar, const Eigen::VectorXd tail, const Eigen::MatrixXd Bu, const Eigen::VectorXd& control_input) {
        G_hat_ = compute_G_hat(G_bar);
        Eigen::VectorXd extended_state(X_hat_.size() + F_hat_.size());
        extended_state << X_hat_, F_hat_;
        extended_state = A_ * extended_state + B_ * G_hat_ + Bu * control_input + tail;
        X_hat_ = extended_state.head(X_hat_.size());
        F_hat_ = extended_state.tail(F_hat_.size());
    }

    // **观测更新**
    void update(const Eigen::VectorXd& Y) {
        Eigen::MatrixXd Kk = compute_Kk();
        Eigen::VectorXd extended_state(X_hat_.size() + F_hat_.size());
        extended_state << X_hat_, F_hat_;
        innovation_ = Y - C_ * extended_state;
        extended_state = extended_state - Kk * innovation_;
        X_hat_ = extended_state.head(X_hat_.size());
        F_hat_ = extended_state.tail(F_hat_.size());
        samples_.push_back(innovation_);
        Ck_ += innovation_ * innovation_.transpose();
        if (samples_.size() > m_) {
            Eigen::VectorXd old_q = samples_.front();
            Ck_ -= old_q * old_q.transpose();
            samples_.pop_front();
        }
        Qa_ = Kk * Ck_/samples_.size() * Kk.transpose();

        // **误差协方差更新**
        P_ = (1 + theta_) * (A_ + Kk * C_) * P_ * (A_ + Kk * C_).transpose() + Kk * R_ * Kk.transpose()
            + (1 + 1 / theta_) * Q1_ + Q2_;
    }

    Eigen::MatrixXd update_P(Eigen::MatrixXd P)
    {
        Eigen::MatrixXd Kk = compute_Kk(P);
        return ( (1 + theta_) * (A_ + Kk * C_) * P * (A_ + Kk * C_).transpose() + Kk * R_ * Kk.transpose()
               + (1 + 1 / theta_) * Q1_ + Q2_ );
    }

    int state_size_;
    int extended_size_;
    int measurement_size_;
    int control_size_;
    Eigen::MatrixXd Qa_; // 预测协方差矩阵
    
private:
    Eigen::VectorXd X_hat_, F_hat_, G_hat_, qk_, innovation_;  // 状态估计、扩展状态估计、非线性估计、已知参数 qk、新息项
    Eigen::MatrixXd P_, A_, B_, Bu_, C_;              // 误差协方差矩阵和系统矩阵
    Eigen::MatrixXd Q1_, Q2_, Qk_, R_, Ck_;           // 噪声协方差矩阵
    std::deque<Eigen::VectorXd> samples_; // 用于存储创新协方差矩阵的队列
    double theta_;                               // 参数 θ
    int m_;                                     // innovation covariance estimation window size
};

class DisturbanceObserver
{
public:
    DisturbanceObserver(double dt);
    DisturbanceObserver() {};   
    ~DisturbanceObserver() {};

    // DiscreteKalmanFilter kalman_filter_{3, 3, 4}; 
    DiscreteKalmanFilter kf_{10, 7, 4};
    ESKF eskf_{10, 3, 4, 10};

    // 更新ESO
    void update(const Eigen::VectorXd& y, double T, 
        double roll, double pitch, double yaw, double m, double g_z);
    
    void update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, 
        double roll, double pitch, double yaw, double m, double g_z, const Eigen::MatrixXd& visual_jacob);

    void finite_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
        double roll, double pitch, double yaw, double m, double g_z, const Eigen::MatrixXd& visual_jacob);

    void nonlinear_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
        double roll, double pitch, double yaw, double m, double g_z, const Eigen::MatrixXd& visual_jacob);

    void kalman_geso_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, double m, double g_z);

    void kalman_gpio_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u, double m, double g_z);

    void update(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
                 double m, double g_z, const Eigen::MatrixXd& visual_jacob);

    void kalman_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
        double roll, double pitch, double yaw, double m, double g_z, const Eigen::MatrixXd& visual_jacob);

    void eskf_update(const Eigen::VectorXd& y, const Eigen::VectorXd& u,
        double m, double g_z);
    
    // 获取估计状态
    Eigen::VectorXd getState() const {
        return z_;
    }

    // 获取估计扰动
    Eigen::VectorXd getDisturbance() const {
        return d_;
    }

    Eigen::VectorXd getDisturbanceFirstOrderDiff() const {
        return dd_;
    }

    void setFirstZ(const Eigen::VectorXd& z) {
        z_ = z;
        // Eigen::VectorXd x0(3);
        // Eigen::MatrixXd P0(3, 3);
        // Eigen::MatrixXd A0(3, 3);
        // Eigen::MatrixXd B0(3, 4);
        // Eigen::MatrixXd H0(3, 3);
        // Eigen::MatrixXd R0(3, 3);
        // Eigen::MatrixXd Q0(3, 3);
        
        // x0 << z_(2), z_(3), z_(4);
        // P0 << 10, 0, 0,
        //     0, 10, 0,
        //     0, 0, 10;
        // Q0 << 0.1, 0, 0,
        //     0, 0.1, 0,
        //     0, 0, 0.1;	
        // R0 << 1, 0, 0,
        //     0, 1, 0,
        //     0, 0, 1;
        // H0 << 1, 0, 0,
        //     0, 1, 0,
        //     0, 0, 1;
        // A0 = Eigen::MatrixXd::Identity(3, 3);
        // B0 = Eigen::MatrixXd::Zero(3, 4);
        // kalman_filter_.init(x0, P0, A0, B0, H0, R0, Q0);
        // Eigen::VectorXd x0(10);
        // x0 << z, 0, 0, 0, 0, 0;
        kf_.init(z_);
        eskf_.init(z_);
    }

    void setDt(double dt) {
        dt_ = dt;
    }

    double getDt() const {
        return dt_;
    }

private:
    double dt_;               // 采样时间
    Eigen::VectorXd L_;                 // Lipschitz常数
    Eigen::VectorXd l_;       // ESO增益
    double pole_;             // ESO极点
    Eigen::VectorXd z_;       // 扩展状态 [p_x, p_y, p_z, v_x, v_y, v_z]
    Eigen::VectorXd d_;       // 扰动 [d_vx, d_vy, d_vz]
    Eigen::VectorXd dd_;
    Eigen::VectorXd ddd_;
    Eigen::VectorXd fl_;      // 有限时间观测器增益
    Eigen::VectorXd fn_;      // 非线性观测器增益 
};

class ContinuousKalmanFilter {
public:
    ContinuousKalmanFilter(int stateSize, int measurementSize) {
        // 初始化状态向量、协方差矩阵等
        x_ = Eigen::VectorXd(stateSize);  // 状态向量
        P_ = Eigen::MatrixXd(stateSize, stateSize);  // 状态协方差矩阵
        A_ = Eigen::MatrixXd(stateSize, stateSize);  // 系统矩阵A
        B_ = Eigen::MatrixXd(stateSize, stateSize);  // 控制矩阵B
        C_ = Eigen::MatrixXd(measurementSize, stateSize);  // 测量矩阵C
        R_ = Eigen::MatrixXd(measurementSize, measurementSize);  // 测量噪声协方差矩阵R
        Q_ = Eigen::MatrixXd(stateSize, stateSize);  // 过程噪声协方差矩阵Q
        I_ = Eigen::MatrixXd::Identity(stateSize, stateSize);  // 单位矩阵
    }

    void init(Eigen::VectorXd initialState, Eigen::MatrixXd initialCovariance,
              Eigen::MatrixXd systemMatrix, Eigen::MatrixXd controlMatrix, 
              Eigen::MatrixXd measurementMatrix, Eigen::MatrixXd measurementNoise, 
              Eigen::MatrixXd processNoise) {
        x_ = initialState;  // 初始状态
        P_ = initialCovariance;  // 初始协方差矩阵
        A_ = systemMatrix;  // 系统矩阵A
        B_ = controlMatrix;  // 控制矩阵B
        C_ = measurementMatrix;  // 测量矩阵C
        R_ = measurementNoise;  // 测量噪声协方差矩阵R
        Q_ = processNoise;  // 过程噪声协方差矩阵Q
    }

    void predict(Eigen::VectorXd controlInput, double dt) {
        // 预测步骤：x = A_ * x_ + B_ * u
        x_ = A_ * x_ + B_ * controlInput;
        
        // 更新协方差矩阵
        P_ = A_ * P_ * A_.transpose() + Q_;
    }

    void update(Eigen::VectorXd measurement) {
        // 更新步骤
        // 计算创新（测量残差）
        Eigen::VectorXd y = measurement - C_ * x_;

        // 计算卡尔曼增益
        Eigen::MatrixXd S = C_ * P_ * C_.transpose() + R_;
        Eigen::MatrixXd K = P_ * C_.transpose() * S.inverse();

        // 更新状态估计
        x_ = x_ + K * y;

        // 更新协方差矩阵
        P_ = (I_ - K * C_) * P_;
    }

    Eigen::VectorXd getState() {
        return x_;
    }

private:
    Eigen::VectorXd x_;  // 状态向量
    Eigen::MatrixXd P_;  // 状态协方差矩阵
    Eigen::MatrixXd A_;  // 系统矩阵A
    Eigen::MatrixXd B_;  // 控制矩阵B
    Eigen::MatrixXd C_;  // 测量矩阵C
    Eigen::MatrixXd R_;  // 测量噪声协方差矩阵R
    Eigen::MatrixXd Q_;  // 过程噪声协方差矩阵Q
    Eigen::MatrixXd I_;  // 单位矩阵
};

