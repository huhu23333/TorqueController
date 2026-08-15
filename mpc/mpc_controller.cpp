#include "mpc_controller.h"

#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <ceres/ceres.h>

// ------------------------------------------------------------
// 辅助函数
// ------------------------------------------------------------
double MPCController::frictionTorque(double omega) const {
    // 软符号斜率：λ=1e4 会使摩擦在 ω≈0 附近过刚，显式欧拉在 dt=1e-3 下发散；
    // 取 λ=100（Stribeck 速度约 0.01 rad/s）在保持 Coulomb 摩擦幅值不变的同时保证数值稳定。
    const double lambda_omega = 100.0;
    double soft_sign = std::tanh(lambda_omega * omega);
    return -soft_sign * tau_c_ - b_ * omega;
}

std::pair<double, double> MPCController::dynamicsStep(double theta, double omega,
                                                      double tau, double dt) const {
    double tau_f = frictionTorque(omega);
    double tau_net = tau + tau_f + tau_d_;
    double alpha = tau_net / J_;
    double omega_new = omega + alpha * dt;
    double theta_new = theta + omega_new * dt;
    return {theta_new, omega_new};
}

void MPCController::predictTrajectory(double theta0, double omega0,
                                      const std::vector<double>& u_seq,
                                      std::vector<double>& theta_pred,
                                      std::vector<double>& omega_pred) const {
    theta_pred.clear();
    omega_pred.clear();
    theta_pred.reserve(N_ + 1);
    omega_pred.reserve(N_ + 1);

    double theta = theta0, omega = omega0;
    theta_pred.push_back(theta);
    omega_pred.push_back(omega);
    for (int k = 0; k < N_; ++k) {
        double tau = u_seq[k];
        for (int step = 0; step < steps_per_control_; ++step) {
            auto p = dynamicsStep(theta, omega, tau, dt_sim_);
            theta = p.first;
            omega = p.second;
        }
        theta_pred.push_back(theta);
        omega_pred.push_back(omega);
    }
}

// ------------------------------------------------------------
// Ceres 代价函数（增量重参数化：d[0]=u[0]，d[k]=u[k]-u[k-1]）
// ------------------------------------------------------------
class MPCCostFunctor {
public:
    MPCCostFunctor(const MPCController* mpc, double theta0, double omega0,
                   const std::vector<double>& theta_ref)
        : mpc_(mpc), theta0_(theta0), omega0_(omega0), theta_ref_(theta_ref) {}

    bool operator()(double const* const* parameters, double* residuals) const {
        const double* d = parameters[0];
        const int N = mpc_->N();

        // 由增量 d 重建力矩序列 u（第一步相对上次实际力矩，含最大力矩硬限幅）
        std::vector<double> u(N);
        u[0] = std::clamp(mpc_->prevTorque() + d[0], -mpc_->maxTorque(), mpc_->maxTorque());
        for (int k = 1; k < N; ++k) {
            double u_k = u[k - 1] + d[k];
            u[k] = std::clamp(u_k, -mpc_->maxTorque(), mpc_->maxTorque());
        }

        std::vector<double> theta_pred, omega_pred;
        mpc_->predictTrajectory(theta0_, omega0_, u, theta_pred, omega_pred);

        int idx = 0;
        // 位置跟踪误差：直接相减，不归一化（多圈连续语义，
        // ref[i] 对应预测 theta_pred[i+1]，i = 0..N-1）
        for (int k = 0; k < N; ++k) {
            double err = theta_pred[k + 1] - theta_ref_[k];
            residuals[idx++] = std::sqrt(mpc_->Q()) * err;
        }
        // 控制量惩罚
        for (int k = 0; k < N; ++k) {
            residuals[idx++] = std::sqrt(mpc_->R()) * u[k];
        }
        // 力矩变化率惩罚（软约束，硬约束由 d[k] 的参数边界保证）
        for (int k = 1; k < N; ++k) {
            residuals[idx++] = std::sqrt(mpc_->Rd()) * d[k];
        }
        return true;
    }

private:
    const MPCController* mpc_;
    double theta0_, omega0_;
    std::vector<double> theta_ref_;
};

// ------------------------------------------------------------
// 构造函数
// ------------------------------------------------------------
MPCController::MPCController(double dt_control, double dt_sim,
                             double J, double tau_c, double b, double tau_d,
                             double max_torque, double max_torque_rate,
                             int N, double Q, double R, double Rd, int max_iter)
    : dt_control_(dt_control), dt_sim_(dt_sim),
      J_(J), tau_c_(tau_c), b_(b), tau_d_(tau_d),
      max_torque_(max_torque), max_torque_rate_(max_torque_rate),
      N_(N), Q_(Q), R_(R), Rd_(Rd), max_iter_(max_iter)
{
    double ratio = dt_control_ / dt_sim_;
    steps_per_control_ = static_cast<int>(std::round(ratio));
    if (std::abs(steps_per_control_ * dt_sim_ - dt_control_) > 1e-9) {
        throw std::invalid_argument("dt_control must be an integer multiple of dt_sim");
    }
    rate_step_ = max_torque_rate_ * dt_control_;
    prev_u_seq_.assign(N_, 0.0);
}

// ------------------------------------------------------------
// 核心 step 方法（同步求解，返回第一步控制量）
// ------------------------------------------------------------
MPCController::Result MPCController::step(double theta, double omega,
                                          const std::vector<double>& theta_ref) {
    if (theta_ref.size() < static_cast<size_t>(N_)) {
        throw std::invalid_argument("theta_ref length must be at least N");
    }
    Result result;
    std::vector<double> ref_copy = theta_ref;

    // 初始猜测：平移上一次最优序列 -> 转为增量 d
    std::vector<double> u_init(N_, 0.0);
    for (int i = 0; i < N_ - 1; ++i) u_init[i] = prev_u_seq_[i + 1];
    if (!prev_u_seq_.empty()) u_init[N_ - 1] = prev_u_seq_.back();

    std::vector<double> d(N_, 0.0);
    d[0] = std::clamp(u_init[0] - prev_torque_, -rate_step_, rate_step_);
    for (int k = 1; k < N_; ++k) {
        d[k] = std::clamp(u_init[k] - u_init[k - 1], -rate_step_, rate_step_);
    }

    ceres::Problem problem;
    MPCCostFunctor* cost_functor = new MPCCostFunctor(this, theta, omega, ref_copy);
    // 残差 = 位置误差(N) + 控制量惩罚(N) + 力矩变化率惩罚(N-1)
    int num_residuals = N_ + N_ + (N_ - 1);
    auto* cost_function = new ceres::DynamicNumericDiffCostFunction<MPCCostFunctor>(
        cost_functor, ceres::TAKE_OWNERSHIP);
    cost_function->SetNumResiduals(num_residuals);
    cost_function->AddParameterBlock(N_);
    problem.AddResidualBlock(cost_function, nullptr, d.data());

    // 参数硬边界：所有增量 d[k] 均受最大力矩变化率限制
    for (int k = 0; k < N_; ++k) {
        problem.SetParameterLowerBound(d.data(), k, -rate_step_);
        problem.SetParameterUpperBound(d.data(), k,  rate_step_);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = max_iter_;
    options.function_tolerance = 1e-8;
    options.parameter_tolerance = 1e-8;
    options.minimizer_progress_to_stdout = false;
    options.num_threads = 1;
    options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // 由最终 d 重建力矩序列
    std::vector<double> u(N_);
    u[0] = std::clamp(prev_torque_ + d[0], -max_torque_, max_torque_);
    for (int k = 1; k < N_; ++k) {
        u[k] = std::clamp(u[k - 1] + d[k], -max_torque_, max_torque_);
    }

    if (!summary.IsSolutionUsable()) {
        // 求解失败：退化为 0 力矩，输出当前状态
        prev_u_seq_.assign(N_, 0.0);
        prev_torque_ = 0.0;
        result.torque = 0.0;
        result.theta = theta;
        result.omega = omega;
        return result;
    }

    prev_u_seq_ = u;
    prev_torque_ = u[0];

    // 第一步预测状态（应用 u[0] 一个控制周期）
    double th = theta, om = omega;
    for (int step = 0; step < steps_per_control_; ++step) {
        auto p = dynamicsStep(th, om, u[0], dt_sim_);
        th = p.first;
        om = p.second;
    }
    result.torque = u[0];
    result.theta = th;
    result.omega = om;
    return result;
}
