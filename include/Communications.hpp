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
#include <string>

// ── 端口筛选函数 ──

// 电控（MCU）：选择不是 IMU 的串口
inline bool mcuPortSelector(const std::string& product_info) {
    return product_info != "STM32 Virtual ComPort MyIMU";
}

// IMU：选择是 IMU 的串口
inline bool imuPortSelector(const std::string& product_info) {
    return product_info == "STM32 Virtual ComPort MyIMU";
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

#endif // COMMUNICATIONS_HPP