// RobotCommunicationC.h — RobotCommunication 的 C 语言 API
//
// 使用方式（Python ctypes / C 程序）:
//   1. robot_comm_create() 创建句柄
//   2. robot_comm_get_latest_data() 获取最新 IMU + MCU 数据
//   3. robot_comm_send_to_mcu / send_to_imu 发送数据
//   4. robot_comm_destroy() 销毁
//
// 注意：所有结构体为 #pragma pack(1)，与 Protocol.hpp 内存布局严格一致

#ifndef ROBOT_COMMUNICATION_C_H
#define ROBOT_COMMUNICATION_C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// C 兼容的数据包结构体（与 Protocol.hpp 中的 C++ 版本内存布局一致）
// ============================================================================

#pragma pack(push, 1)

// ── MCU 发送包 ──
typedef struct {
    uint8_t frame_header1;       // = 0x42
    uint8_t frame_header2;       // = 0x52
    uint8_t protocol_version;    // = 0x01
    uint8_t data_size;           // = 10
    uint8_t auto_aim_enable;
    float   pitch_target_angle;
    float   yaw_torque;
    uint8_t fire;
    uint8_t crc8;
} McuSendPacket_C;

// ── MCU 接收包 ──
typedef struct {
    uint8_t frame_header1;       // = 0x42
    uint8_t frame_header2;       // = 0x52
    uint8_t protocol_version;    // = 0x01
    uint8_t data_size;
    float   bullet_velocity;
    float   pitch_angle;
    float   yaw_angle;
    float   yaw_omega;
    float   chassis_imu_yaw;
    float   chassis_imu_omega;
    uint8_t mark;
    uint8_t color;
    uint8_t auto_aim_switch;
    uint8_t crc8;
} McuReceivePacket_C;

// ── IMU 发送包 ──
typedef struct {
    uint8_t  frame_header1;      // = 0xA7
    uint8_t  frame_header2;      // = 0xB6
    uint8_t  frame_header3;      // = 0xC5
    uint8_t  data_size;          // = 0
    uint32_t crc32;
} ImuSendPacket_C;

// ── IMU 接收包 ──
typedef struct {
    uint8_t  frame_header1;      // = 0xA7
    uint8_t  frame_header2;      // = 0xB6
    uint8_t  frame_header3;      // = 0xC5
    uint8_t  data_size;
    float    gx;
    float    gy;
    float    gz;
    float    ax;
    float    ay;
    float    az;
    double   euler_yaw;
    double   euler_pitch;
    double   euler_roll;
    uint32_t dt_one_tenth_ms;
    uint32_t crc32;
} ImuReceivePacket_C;

#pragma pack(pop)

// ── 聚合数据 ──
typedef struct {
    bool              imu_valid;
    ImuReceivePacket_C imu_packet;
    bool              mcu_valid;
    McuReceivePacket_C mcu_packet;
} RobotLatestData_C;

// ── 不透明句柄 ──
typedef struct RobotCommHandle RobotCommHandle;

// ============================================================================
// API 函数
// ============================================================================

// 创建通信句柄（自动启动 IMU 和 MCU 的串口监听）
RobotCommHandle* robot_comm_create(void);

// 销毁通信句柄
void robot_comm_destroy(RobotCommHandle* handle);

// 获取最新 IMU 和 MCU 数据（MCU 接收数据会在此处做预处理）
RobotLatestData_C robot_comm_get_latest_data(RobotCommHandle* handle);

// 发送 MCU 数据（发送前做预处理）
bool robot_comm_send_to_mcu(RobotCommHandle* handle, McuSendPacket_C packet);

// 发送 IMU 数据（无预处理）
bool robot_comm_send_to_imu(RobotCommHandle* handle, ImuSendPacket_C packet);

// 停止通信
void robot_comm_stop(RobotCommHandle* handle);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_COMMUNICATION_C_H
