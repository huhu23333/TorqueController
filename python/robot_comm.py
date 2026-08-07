"""
robot_comm.py — RobotCommunication 的 ctypes Python 接口
"""

import ctypes
import os
from ctypes import (
    Structure, c_uint8, c_float, c_double, c_uint32, c_bool,
)

def _find_lib():
    candidates = [
        os.path.join(os.path.dirname(__file__), "..", "build", "librobot_comm_c.so"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return "librobot_comm_c.so"

_lib = ctypes.CDLL(_find_lib())


class McuSendPacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1",       c_uint8),
        ("frame_header2",       c_uint8),
        ("protocol_version",    c_uint8),
        ("data_size",           c_uint8),
        ("auto_aim_enable",     c_uint8),
        ("pitch_target_angle",  c_float),
        ("yaw_torque",          c_float),
        ("fire",                c_uint8),
        ("crc8",                c_uint8),
    ]

    def __init__(self, auto_aim_enable=1, pitch_target_angle=0.0, yaw_torque=0.0, fire=0):
        super().__init__()
        self.frame_header1 = 0x42
        self.frame_header2 = 0x52
        self.protocol_version = 0x01
        self.data_size = 10
        self.auto_aim_enable = auto_aim_enable
        self.pitch_target_angle = pitch_target_angle
        self.yaw_torque = yaw_torque
        self.fire = fire
        self.crc8 = 0


class McuReceivePacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1",     c_uint8),
        ("frame_header2",     c_uint8),
        ("protocol_version",  c_uint8),
        ("data_size",         c_uint8),
        ("bullet_velocity",   c_float),
        ("pitch_angle",       c_float),
        ("yaw_angle",         c_float),
        ("yaw_omega",         c_float),
        ("chassis_imu_yaw",   c_float),
        ("chassis_imu_omega", c_float),
        ("mark",              c_uint8),
        ("color",             c_uint8),
        ("auto_aim_switch",   c_uint8),
        ("crc8",              c_uint8),
    ]


class ImuSendPacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1", c_uint8),
        ("frame_header2", c_uint8),
        ("frame_header3", c_uint8),
        ("data_size",     c_uint8),
        ("crc32",         c_uint32),
    ]

    def __init__(self):
        super().__init__()
        self.frame_header1 = 0xA7
        self.frame_header2 = 0xB6
        self.frame_header3 = 0xC5
        self.data_size = 0
        self.crc32 = 0


class ImuReceivePacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1",   c_uint8),
        ("frame_header2",   c_uint8),
        ("frame_header3",   c_uint8),
        ("data_size",       c_uint8),
        ("gx",              c_float),
        ("gy",              c_float),
        ("gz",              c_float),
        ("ax",              c_float),
        ("ay",              c_float),
        ("az",              c_float),
        ("euler_yaw",       c_double),
        ("euler_pitch",     c_double),
        ("euler_roll",      c_double),
        ("dt_one_tenth_ms", c_uint32),
        ("crc32",           c_uint32),
    ]


class RobotLatestData(Structure):
    _pack_ = 1
    _fields_ = [
        ("imu_valid",  c_bool),
        ("imu_packet", ImuReceivePacket),
        ("mcu_valid",  c_bool),
        ("mcu_packet", McuReceivePacket),
    ]


# ============================================================================
# 函数签名
# ============================================================================

_lib.robot_comm_create.restype             = ctypes.c_void_p
_lib.robot_comm_destroy.argtypes           = [ctypes.c_void_p]
_lib.robot_comm_get_latest_data.argtypes   = [ctypes.c_void_p]
_lib.robot_comm_get_latest_data.restype    = RobotLatestData
_lib.robot_comm_send_to_mcu.argtypes       = [ctypes.c_void_p, McuSendPacket]
_lib.robot_comm_send_to_mcu.restype        = c_bool
_lib.robot_comm_send_to_imu.argtypes       = [ctypes.c_void_p, ImuSendPacket]
_lib.robot_comm_send_to_imu.restype        = c_bool
_lib.robot_comm_stop.argtypes              = [ctypes.c_void_p]


# ============================================================================
# Python 封装类
# ============================================================================

class RobotCommunication:
    """RobotCommunication 的 Python 接口"""

    def __init__(self):
        self._handle = _lib.robot_comm_create()
        if not self._handle:
            raise RuntimeError("robot_comm_create() returned NULL")

    def get_latest_data(self) -> RobotLatestData:
        """获取最新 IMU 和 MCU 数据（MCU 数据已预处理）"""
        return _lib.robot_comm_get_latest_data(self._handle)

    def send_to_mcu(self, packet: McuSendPacket) -> bool:
        """发送 MCU 数据（发送前预处理）"""
        return _lib.robot_comm_send_to_mcu(self._handle, packet)

    def send_to_imu(self, packet: ImuSendPacket) -> bool:
        """发送 IMU 数据（无预处理）"""
        return _lib.robot_comm_send_to_imu(self._handle, packet)

    def stop(self):
        """停止通信 worker 线程"""
        _lib.robot_comm_stop(self._handle)

    def close(self):
        """销毁句柄，释放资源"""
        if self._handle:
            _lib.robot_comm_destroy(self._handle)
            self._handle = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.stop()
        self.close()

    def __del__(self):
        self.close()

