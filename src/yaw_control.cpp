// yaw_control.cpp
// Yaw 轴 PID 闭环控制程序
// - 目标角度：来自 IMU 的 euler_yaw（imu::ReceivePacket）
// - 观测角度：来自 MCU 接收包经 McuDataPreprocessor 预处理后的 yaw_angle
// - PID 输出 yaw_torque → 通过 mcu::SendPacket 发送给电控

#include "Communications.hpp"
#include "McuDataPreprocessor.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <mutex>
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

std::mutex         imu_mutex_;
imu::ReceivePacket latest_imu_packet_{};
bool               has_imu_data_ = false;

std::mutex         mcu_mutex_;
mcu::ReceivePacket latest_mcu_packet_{};
bool               has_mcu_data_ = false;

// ============================================================================
// 回调函数
// ============================================================================

void onImuReceive(const imu::ReceivePacket& packet) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    latest_imu_packet_ = packet;
    has_imu_data_ = true;
}

void onMcuReceive(const mcu::ReceivePacket& packet) {
    mcu::ReceivePacket processed = McuDataPreprocessor::processReceive(packet);
    std::lock_guard<std::mutex> lock(mcu_mutex_);
    latest_mcu_packet_ = processed;
    has_mcu_data_ = true;
}

float normalizeAngle(float angle) {
    while (angle >  M_PI) angle -= 2.0f * static_cast<float>(M_PI);
    while (angle < -M_PI) angle += 2.0f * static_cast<float>(M_PI);
    return angle;
}

void signalHandler(int) {
    keep_running = false;
}

} // namespace

int main() {
    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "========================================" << std::endl;
    std::cout << "  Yaw 轴 PID 闭环控制程序" << std::endl;
    std::cout << "  目标: IMU euler_yaw" << std::endl;
    std::cout << "  观测: MCU yaw_angle (预处理后)" << std::endl;
    std::cout << "========================================" << std::endl;

    ImuCommunication imu_serial(onImuReceive);
    McuCommunication mcu_serial(onMcuReceive);

    std::cout << "IMU 和 MCU 通信已启动，等待数据..." << std::endl;

    // PID: Kp=2.0 Ki=0.1 Kd=0.05, output [-1.0, 1.0]
    PidController pid(2.0f, 0.1f, 0.05f, -1.0f, 1.0f);

    auto last_time = std::chrono::steady_clock::now();
    int  loop_count = 0;

    while (keep_running) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        float target_yaw = 0.0f;
        bool  imu_valid  = false;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (has_imu_data_) {
                target_yaw = static_cast<float>(latest_imu_packet_.euler_yaw);
                imu_valid  = true;
            }
        }

        float observed_yaw = 0.0f;
        bool  mcu_valid    = false;
        {
            std::lock_guard<std::mutex> lock(mcu_mutex_);
            if (has_mcu_data_) {
                observed_yaw = latest_mcu_packet_.yaw_angle;
                mcu_valid    = true;
            }
        }

        float yaw_torque = 0.0f;
        if (imu_valid && mcu_valid) {
            float error = normalizeAngle(target_yaw - observed_yaw);
            yaw_torque = pid.update(error, dt);
        }

        // 构造发送包并发送
        mcu::SendPacket pkt;
        pkt.auto_aim_enable    = 1;
        pkt.pitch_target_angle = 10.0f;
        pkt.yaw_torque         = yaw_torque;
        pkt.fire               = 0;
        mcu_serial.sendData(pkt);

        // 每 100 次循环打印一次状态
        if (++loop_count % 100 == 0) {
            float err = 0.0f;
            if (imu_valid && mcu_valid)
                err = normalizeAngle(target_yaw - observed_yaw);
            std::cout << std::fixed << std::setprecision(3)
                      << "[Loop " << loop_count << "] "
                      << "target=" << target_yaw
                      << " observed=" << observed_yaw
                      << " error=" << err
                      << " torque=" << yaw_torque
                      << " (IMU:" << (imu_valid ? "Y" : "N")
                      << " MCU:" << (mcu_valid ? "Y" : "N") << ")"
                      << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\n程序退出。" << std::endl;

    imu_serial.stopWorker();
    mcu_serial.stopWorker();

    return 0;
}

