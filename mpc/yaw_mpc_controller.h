#ifndef YAW_MPC_CONTROLLER_H
#define YAW_MPC_CONTROLLER_H

#include <deque>
#include <vector>
#include "Communications.hpp"
#include "mpc_controller.h"

// ============================================================================
// YawMpcController — 实车 yaw 轴 MPC 求解封装（参考序列 + 求解，不发送）
//
// - 初始化: 控制周期 dt_control、预测步数 N、MPC 参数（辨识 + 约束 + 权重）
// - 每步调用 step(target_yaw)（只传目标位置），内部完成:
//     1) 读取融合状态（yaw_pos / yaw_rate / imu_yaw_unwrapped）与
//        MCU 的 chassis_imu_omega（底盘角速度，参考外推用）
//     2) 维护延迟参考序列: 目标延迟 dt_control*N（buffer 保持最多 N 个），
//        ref[i] = 延迟目标[i] − ((theta_imu − theta) + (i+1)·dt·chassis_imu_omega)
//     3) MPC 求解第一步控制量
// - 返回将要发送的值（yaw_target_angle / yaw_target_velocity / yaw_torque），
//   由上层封装决定如何发送
// ============================================================================
class YawMpcController {
public:
    struct Result {
        double yaw_target_angle = 0.0;    // 预测位置 → 发送 yaw_target_angle (rad)
        double yaw_target_velocity = 0.0; // 预测速度 → 发送 yaw_target_velocity (rad/s)
        double yaw_torque = 0.0;          // 控制力矩 → 发送 yaw_torque (N·m)
        double delayed_target = 0.0;      // 当前参考（延迟 N 步的目标），供显示
    };

    // dt_control: 控制周期（MPC 的 dt_control 与 dt_sim 相同）
    // N: 预测步数
    YawMpcController(RobotCommunication* comm, double dt_control, int N,
                     double J, double tau_c, double b, double tau_d,
                     double max_torque, double max_torque_rate,
                     double Q, double R, double Rd, int max_iter);

    // 每步调用：内部读状态、维护参考序列、求解；返回发送所需值（不发送）
    Result step(double target_yaw);

private:
    RobotCommunication* comm_;
    double dt_;
    int    N_;
    MPCController mpc_;
    std::deque<double> target_buf_;   // 延迟目标序列（最多 N 个，最旧在前）
};

#endif // YAW_MPC_CONTROLLER_H
