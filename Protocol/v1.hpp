#pragma once
#include <stdint.h>

#pragma pack(push, 1)
struct SendPacket
{
    uint8_t frame_header1 = 0x42;
    uint8_t frame_header2 = 0x52;
    uint8_t protocol_version = 0x01;
    uint8_t data_size = 10;             // auto_aim_enable(1) + pitch_target_angle(4) + yaw_torque(4) + fire(1)
    uint8_t auto_aim_enable;            // 和之前的reset相反
    float pitch_target_angle;           // -pi/2 ~ pi/2
    float yaw_torque;                   // 单位要测完之后定
    uint8_t fire;
    uint8_t crc8;
};


struct ReceivePacket
{
    uint8_t frame_header1 = 0x42;
    uint8_t frame_header2 = 0x52;
    uint8_t protocol_version = 0x01;
    uint8_t data_size;
    float bullet_velocity;              // m/s
    float pitch_angle;                  // -pi/2 ~ pi/2
    float yaw_angle;                    // 0 ~ 2pi ，这里不要减掉imu的角度，直接读yaw轴电机编码器的角度
    float yaw_omega;                    // rad/s ，yaw轴电机编码器读到的角速度
    float chassis_imu_yaw;              // 0 ~ 2pi ，底盘imu积分的yaw轴角度
    float chassis_imu_omega;            // 0 ~ 2pi ，底盘imu的yaw轴角速度
    uint8_t mark;                       // 原递增循环标志位
    uint8_t color;                      // 原颜色标志位
    uint8_t auto_aim_switch;            // 电控的自瞄开关
    uint8_t crc8;
};
#pragma pack(pop)