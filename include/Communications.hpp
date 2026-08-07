// Communications.hpp — 基于 SerialProtocol 模板的具体通信类型定义
//
// McuCommunication : 与电控（MCU）通信，CRC8，前导 0x42 0x52 0x01
// ImuCommunication  : 与 IMU 模块通信，CRC32，前导 0xA7 0xB6 0xC5
//
#ifndef COMMUNICATIONS_HPP
#define COMMUNICATIONS_HPP

#include "SerialProtocol.hpp"
#include "Protocol.hpp"
#include "CRC.h"
#include "McuDataPreprocessor.hpp"
#include <string>
#include <mutex>

// ── 端口筛选函数 ──

// 电控（MCU）：选择不是 IMU 的串口
inline bool mcuPortSelector(const std::string& product_info) {
    return product_info != "AutoAim_IMU_Com";
}

// IMU：选择是 IMU 的串口
inline bool imuPortSelector(const std::string& product_info) {
    return product_info == "AutoAim_IMU_Com";
}

// ── 具体通信类型别名 ──

// 电控（MCU）通信：CRC8，前导字节见 mcu::SendPacket / mcu::ReceivePacket
using McuCommunication = SerialProtocol<
    mcu::SendPacket,
    mcu::ReceivePacket,
    CRC8_Check_Sum,
    mcuPortSelector,
    mcu::PREAMBLE_SIZE
>;

// IMU 通信：CRC32，前导字节见 imu::SendPacket / imu::ReceivePacket
using ImuCommunication = SerialProtocol<
    imu::SendPacket,
    imu::ReceivePacket,
    CRC32_Calculate,
    imuPortSelector,
    imu::PREAMBLE_SIZE
>;

// ============================================================================
// RobotCommunication — 组合 MCU 与 IMU 通信，封装数据预处理
// ============================================================================
// - 回调中直接存储原始数据，不做预处理
// - McuDataPreprocessor 仅在获取数据 (getLatestData) 或发送 MCU 数据 (sendToMcu) 时使用
// - 提供统一的最新数据获取接口和分离的发送接口
// ============================================================================
class RobotCommunication {
public:
    struct LatestData {
        bool               imu_valid = false;
        imu::ReceivePacket imu_packet{};
        bool               mcu_valid = false;
        mcu::ReceivePacket mcu_packet{};
    };

    RobotCommunication()
        : mcu_serial_([this](const mcu::ReceivePacket& pkt) { onMcuReceive(pkt); }, false)
        , imu_serial_([this](const imu::ReceivePacket& pkt) { onImuReceive(pkt); }, false)
    {
        mcu_serial_.startWorker();
        imu_serial_.startWorker();
    }

    ~RobotCommunication() {
        mcu_serial_.stopWorker();
        imu_serial_.stopWorker();
    }

    // 获取最新数据（MCU 接收数据在此预处理）
    LatestData getLatestData() {
        LatestData data;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            if (has_imu_data_) {
                data.imu_packet = latest_imu_packet_;
                data.imu_valid  = true;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mcu_mutex_);
            if (has_mcu_data_) {
                data.mcu_packet = McuDataPreprocessor::processReceive(latest_mcu_packet_);
                data.mcu_valid  = true;
            }
        }
        return data;
    }

    // 发送 MCU 数据（发送前预处理）
    bool sendToMcu(mcu::SendPacket packet) {
        mcu::SendPacket processed = McuDataPreprocessor::processSend(packet);
        return mcu_serial_.sendData(processed);
    }

    // 发送 IMU 数据（心跳等，无预处理）
    bool sendToImu(imu::SendPacket packet) {
        return imu_serial_.sendData(packet);
    }

    void stop() {
        mcu_serial_.stopWorker();
        imu_serial_.stopWorker();
    }

private:
    // ── 回调：仅存储原始数据，不做预处理 ──
    void onImuReceive(const imu::ReceivePacket& packet) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_imu_packet_ = packet;
        has_imu_data_      = true;
    }

    void onMcuReceive(const mcu::ReceivePacket& packet) {
        std::lock_guard<std::mutex> lock(mcu_mutex_);
        latest_mcu_packet_ = packet;   // 原始数据，预处理推迟到 getLatestData()
        has_mcu_data_      = true;
    }

    // ── 成员变量 ──
    McuCommunication mcu_serial_;
    ImuCommunication imu_serial_;

    std::mutex         imu_mutex_;
    imu::ReceivePacket latest_imu_packet_{};
    bool               has_imu_data_ = false;

    std::mutex         mcu_mutex_;
    mcu::ReceivePacket latest_mcu_packet_{};
    bool               has_mcu_data_ = false;
};

#endif // COMMUNICATIONS_HPP