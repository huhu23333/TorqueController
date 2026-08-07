#include "McuDataPreprocessor.hpp"
#include <cmath>

mcu::SendPacket McuDataPreprocessor::processSend(const mcu::SendPacket& packet) {
    mcu::SendPacket result = packet;
    result.pitch_target_angle = packet.pitch_target_angle; // 恒等变换占位
    return result;
}

mcu::ReceivePacket McuDataPreprocessor::processReceive(const mcu::ReceivePacket& packet) {
    static constexpr float YAW_SCALE = 2.0f * M_PI / 8192.0f;

    mcu::ReceivePacket result = packet;
    result.pitch_angle = packet.pitch_angle;                // 恒等变换占位
    result.yaw_angle   = packet.yaw_angle * YAW_SCALE;      // 编码器值 → 弧度
    return result;
}