#include "mpc/yaw_mpc_controller.h"

YawMpcController::YawMpcController(RobotCommunication* comm, double dt_control, int N,
                                   double J, double tau_c, double b, double tau_d,
                                   double max_torque, double max_torque_rate,
                                   double Q, double R, double Rd, int max_iter)
    : comm_(comm), dt_(dt_control), N_(N),
      // 与实车脚本一致：MPC 的 dt_sim = dt_control（每个控制周期一个仿真子步）
      mpc_(dt_control, dt_control, J, tau_c, b, tau_d,
           max_torque, max_torque_rate, N, Q, R, Rd, max_iter)
{}

YawMpcController::Result YawMpcController::step(double target_yaw) {
    Result r;
    if (!comm_) return r;

    // ---- 1. 读取融合状态 ----
    auto fused = comm_->getFused();
    if (!fused.valid) return r;   // 融合未就绪

    const double theta     = fused.yaw_pos;           // yaw 关节解卷绕位置
    const double omega     = fused.yaw_rate;          // yaw 关节速度（高频）
    const double theta_imu = fused.imu_yaw_unwrapped; // IMU yaw 解卷绕（world 系）

    // 底盘角速度（MCU 原始数据；参考序列外推用）
    double chassis_imu_omega = 0.0;
    {
        auto data = comm_->getLatestData();
        if (data.mcu_valid) chassis_imu_omega = data.mcu_packet.chassis_imu_omega;
    }

    // ---- 2. 目标延迟缓冲（最多 N 个，延迟 dt*N）----
    target_buf_.push_back(target_yaw);
    while (target_buf_.size() > static_cast<size_t>(N_)) target_buf_.pop_front();
    r.delayed_target = target_buf_.front();

    // ---- 3. 参考序列（与实车 Python 逻辑一致）----
    // ref[i] = 延迟目标[i] − ((theta_imu − theta) + (i+1)·dt·chassis_imu_omega)
    std::vector<double> ref;
    ref.reserve(N_);
    for (size_t i = 0; i < target_buf_.size(); ++i) {
        ref.push_back(target_buf_[i] -
                      ((theta_imu - theta) + (i + 1) * dt_ * chassis_imu_omega));
    }
    while (ref.size() < static_cast<size_t>(N_)) ref.push_back(ref.back());

    // ---- 4. MPC 求解（返回发送所需值 + 参考/预测序列，不发送）----
    auto mres = mpc_.step(theta, omega, ref);
    r.yaw_target_angle    = mres.theta;
    r.yaw_target_velocity = mres.omega;
    r.yaw_torque          = mres.torque;
    r.ref_sequence        = mpc_.lastRef();     // 本次参考（目标）序列（N 个）
    r.pred_sequence       = mpc_.lastPred();    // 本次预测位置序列（N+1 个）

    return r;
}
