// control_demo.cpp
// 控制演示程序 (原 yaw_control)
// - 目标角度：来自 IMU 的 euler_yaw（观测值与目标值相等，显式赋值）
// - 观测角度：来自 IMU 的 euler_yaw（imu::ReceivePacket）
// - Pitch 目标：来自 IMU 的 euler_pitch
// - PID 输出 yaw_torque → 通过 mcu::SendPacket 发送给电控

#include "Communications.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <atomic>
#include <csignal>
#include <chrono>

namespace {

// ============================================================================
// 简易 PID 控制器
// ============================================================================
class PidController {
public:
    PidController(float kp, float ki, float kd, float output_min, float output_max)
        : kp_(kp), ki_(ki), kd_(kd)
        , output_min_(output_min), output_max_(output_max)
    {}

    float update(float error, float dt) {
        float derivative = (dt > 1e-6f) ? (error - prev_error_) / dt : 0.0f;
        prev_error_ = error;

        // 先算 PD 分量，再加积分项预估是否越界
        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        bool  sat_hi = (output > output_max_);
        bool  sat_lo = (output < output_min_);

        if (sat_hi) output = output_max_;
        if (sat_lo) output = output_min_;

        // 条件积分：仅在未饱和 或 误差方向利于退出饱和时累加
        bool do_integrate = true;
        if (sat_hi && error > 0.0f) do_integrate = false;  // 饱和上界且误差继续推高 → 抑制
        if (sat_lo && error < 0.0f) do_integrate = false;  // 饱和下界且误差继续压低 → 抑制
        if (do_integrate) integral_ += error * dt;

        return output;
    }

    void reset() { integral_ = 0.0f; prev_error_ = 0.0f; }
    void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }

private:
    float kp_, ki_, kd_;
    float output_min_, output_max_;
    float integral_   = 0.0f;
    float prev_error_ = 0.0f;
};

// ============================================================================
// 全局变量
// ============================================================================
std::atomic<bool> keep_running{true};

const double TWO_PI = 2.0 * M_PI;
double normalizeAngle(double angle) {
    return std::remainder(angle, TWO_PI);
}

void signalHandler(int) {
    keep_running = false;
}

} // namespace

int main() {
    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "========================================" << std::endl;
    std::cout << "  控制演示程序 (control_demo)" << std::endl;
    std::cout << "  目标: IMU euler_yaw (与观测值相等)" << std::endl;
    std::cout << "  观测: IMU euler_yaw" << std::endl;
    std::cout << "  Pitch目标: IMU euler_pitch" << std::endl;
    std::cout << "========================================" << std::endl;

    RobotCommunication robot_comm;

    std::cout << "IMU 和 MCU 通信已启动，等待数据..." << std::endl;

    // PID: Kp=2.0 Ki=0.1 Kd=0.05, output [-1.0, 1.0]
    PidController pid(2.0f, 0.1f, 0.2f, -1.0f, 1.0f);

    // 等待直到 IMU 和 MCU 均有有效数据
    while (keep_running) {
        auto data = robot_comm.getLatestData();
        if (data.imu_valid && data.mcu_valid) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (keep_running) std::cout << "已收到 IMU 和 MCU 数据，开始控制循环。" << std::endl;

    auto last_time = std::chrono::steady_clock::now();
    int  loop_count = 0;
    while (keep_running) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        // 统一获取最新 IMU 和 MCU 数据
        auto data = robot_comm.getLatestData();

        float target_yaw   = 0.0f;
        float observed_yaw = 0.0f;
        if (data.imu_valid) {
            target_yaw   = static_cast<float>(data.imu_packet.euler_yaw);
            observed_yaw = static_cast<float>(data.imu_packet.euler_yaw);
        }

        float yaw_torque = 0.0f;
        if (data.imu_valid) {
            float error = normalizeAngle(target_yaw - observed_yaw);
            yaw_torque = pid.update(error, dt);
        }

        // 构造发送包并发送（预处理在 sendToMcu 内部完成）
        mcu::SendPacket pkt;
        pkt.auto_aim_enable    = 1;
        pkt.pitch_target_angle = data.imu_valid ? static_cast<float>(data.imu_packet.euler_pitch) : 0.0f;
        pkt.yaw_torque         = yaw_torque;
        pkt.fire               = 0;
        robot_comm.sendToMcu(pkt);

        // 每 100 次循环打印一次状态
        if (++loop_count % 100 == 0) {
            float err = 0.0f;
            if (data.imu_valid)
                err = normalizeAngle(target_yaw - observed_yaw);
            std::cout << std::fixed << std::setprecision(3)
                      << "[Loop " << loop_count << "] "
                      << "target=" << target_yaw
                      << " observed=" << observed_yaw
                      << " error=" << err
                      << " torque=" << yaw_torque
                      << " pitch_target=" << pkt.pitch_target_angle
                      << " (IMU:" << (data.imu_valid ? "Y" : "N") << ")"
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\n程序退出。" << std::endl;

    robot_comm.stop();

    return 0;
}
