#include "communication/McuDataPreprocessor.h"
#include <cmath>

mcu::SendPacket McuDataPreprocessor::processSend(const mcu::SendPacket& packet) {
    mcu::SendPacket result = packet;
    result.pitch_target_angle = -19.413256 * packet.pitch_target_angle + 10.990499; // imu_euler_pitch → pitch_target_angle
    return result;
}

mcu::ReceivePacket McuDataPreprocessor::processReceive(const mcu::ReceivePacket& packet) {
    // static constexpr float YAW_SCALE = 2.0f * M_PI / 8192.0f;

    mcu::ReceivePacket result = packet;
    result.pitch_angle = -1.140968 * packet.pitch_angle + 0.714840;                 // mcu_pitch_angle → imu_euler_pitch
    result.yaw_angle   = packet.yaw_angle;// * YAW_SCALE;                              // 编码器值 → 弧度
    return result;
}