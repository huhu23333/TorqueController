#include "mcu_mpc_controller.h"

#include <chrono>
#include <cmath>

McuMpcController::McuMpcController(RobotCommunication* comm, double dt_control, int N,
                                   double J, double tau_c, double b, double tau_d,
                                   double max_torque, double max_torque_rate,
                                   double Q, double R, double Rd, int max_iter)
    : comm_(comm),
      mpc_(comm, dt_control, N,
           J, tau_c, b, tau_d,
           max_torque, max_torque_rate,
           Q, R, Rd, max_iter)
{}

McuMpcController::~McuMpcController() {
    stop();
}

void McuMpcController::start() {
    if (!running_.exchange(true)) {
        thread_ = std::thread(&McuMpcController::loop, this);
    }
}

void McuMpcController::stop() {
    if (running_.exchange(false)) {
        if (thread_.joinable()) thread_.join();
    }
}

void McuMpcController::set(bool auto_aim_enable, bool yaw_torque_only_mode,
                           double target_yaw, double pitch_target_angle, bool fire) {
    // target_yaw 自动转换到与 imu_yaw_unwrapped 角度差最小的等效角：
    //   target_adj = imu_yaw + remainder(target_yaw − imu_yaw, 2π)
    //   - |target_adj − imu_yaw_unwrapped| ≤ π（角度差最小）
    //   - target_adj ≡ target_yaw (mod 2π)（与 target_yaw 同向/同角度）
    //   避免参考序列引入整圈偏差
    double imu_yaw = 0.0;
    if (comm_) {
        auto fused = comm_->getFused();
        if (fused.valid) imu_yaw = fused.imu_yaw_unwrapped;
    }
    double target_adj = imu_yaw + std::remainder(target_yaw - imu_yaw, 2.0 * M_PI);

    std::lock_guard<std::mutex> lock(set_mtx_);
    auto_aim_enable_      = auto_aim_enable;
    yaw_torque_only_mode_ = yaw_torque_only_mode;
    target_yaw_           = target_adj;
    pitch_target_angle_   = pitch_target_angle;
    fire_                 = fire;
}

McuMpcController::State McuMpcController::state() const {
    std::lock_guard<std::mutex> lock(state_mtx_);
    return last_state_;
}

void McuMpcController::loop() {
    while (running_) {
        // 循环开始处获取本次循环时间基准
        auto start = std::chrono::steady_clock::now();

        // 取最新设置的发送参数与 mpc 目标
        bool aa, mode, fire;
        double target_yaw, pitch;
        {
            std::lock_guard<std::mutex> lock(set_mtx_);
            aa    = auto_aim_enable_;
            mode  = yaw_torque_only_mode_;
            target_yaw = target_yaw_;
            pitch = pitch_target_angle_;
            fire  = fire_;
        }

        // mpc 求解（内部读融合状态）
        auto res = mpc_.step(target_yaw);

        // 配合最新设置构造发送包
        mcu::SendPacket pkt;
        pkt.auto_aim_enable    = aa ? 1 : 0;
        pkt.fire               = fire ? 1 : 0;
        pkt.pitch_target_angle = static_cast<float>(pitch);
        pkt.yaw_torque_only_mode = mode ? 1 : 0;
        pkt.yaw_target_angle   = res.yaw_target_angle;
        pkt.yaw_target_velocity = static_cast<float>(res.yaw_target_velocity);
        pkt.yaw_torque         = static_cast<float>(res.yaw_torque);
        if (comm_) comm_->sendToMcu(pkt);

        // 缓存最新结果（供显示）
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            last_state_.yaw_target_angle    = res.yaw_target_angle;
            last_state_.yaw_target_velocity = res.yaw_target_velocity;
            last_state_.yaw_torque          = res.yaw_torque;
            last_state_.delayed_target      = res.delayed_target;
        }

        // 循环结束处：等待到 start + 10ms（100Hz），不严格跟随绝对时间点
        std::this_thread::sleep_until(start + std::chrono::milliseconds(10));
    }
}
