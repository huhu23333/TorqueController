#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <vector>
#include <utility>

// ============================================================================
// MPCController — 偏航轴 MPC 求解器（Ceres）
//
// 模型: J * dω/dt = τ - τ_c * sign(ω) - b * ω + τ_d
//   （sign 用 tanh 软符号近似，与 param_ident 一致）
//
// 限制:
//   - 最大力矩   max_torque      (N·m)，参数硬边界
//   - 最大力矩变化率 max_torque_rate (N·m/s)，通过增量重参数化硬约束
//   - max_omega 参数保留（接口兼容），当前版本不施加任何速度限制
//
// 参考轨迹: theta_ref 为未来 N 个控制周期（长度 N）的多圈连续参考角度，
//   ref[i] 对应预测 theta_pred[i+1]；位置误差直接相减（不归一化到 (-π, π]），
//   保留圈数语义——目标与当前状态差多少圈就跟踪多少圈。
// ============================================================================
class MPCController {
public:
    struct Result {
        double torque = 0.0;  // 第一步控制力矩
        double theta  = 0.0;  // 第一步预测位置 (rad)
        double omega  = 0.0;  // 第一步预测速度 (rad/s)
    };

    MPCController(double dt_control, double dt_sim,
                  double J, double tau_c, double b, double tau_d,
                  double max_omega, double max_torque, double max_torque_rate,
                  int N, double Q, double R, double Rd, int max_iter);

    Result step(double theta, double omega, const std::vector<double>& theta_ref);

    // 供 CostFunctor 调用
    void predictTrajectory(double theta0, double omega0,
                           const std::vector<double>& u_seq,
                           std::vector<double>& theta_pred,
                           std::vector<double>& omega_pred) const;

    // 供 CostFunctor 读取
    int    N()          const { return N_; }
    double Q()          const { return Q_; }
    double R()          const { return R_; }
    double Rd()         const { return Rd_; }
    double maxTorque()  const { return max_torque_; }
    double rateStep()   const { return rate_step_; }
    double prevTorque() const { return prev_torque_; }

private:
    double dt_control_, dt_sim_;
    int    steps_per_control_;
    double J_, tau_c_, b_, tau_d_;
    double max_omega_, max_torque_, max_torque_rate_;   // max_omega_ 当前未使用（预留）
    double rate_step_;   // 每个控制步允许的最大力矩增量 = max_torque_rate * dt_control
    int    N_;
    double Q_, R_, Rd_;
    int    max_iter_;

    // 上一次最优控制序列（用于热启动）
    std::vector<double> prev_u_seq_;
    // 上一次实际施加的力矩（用于第一步力矩变化率硬约束）
    double prev_torque_ = 0.0;

    double frictionTorque(double omega) const;
    std::pair<double, double> dynamicsStep(double theta, double omega,
                                           double tau, double dt) const;
};

#endif // MPC_CONTROLLER_H
