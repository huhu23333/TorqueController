"""_bridge.py — ctypes 底层桥接，不对外暴露"""

import ctypes
import os
from ctypes import Structure, c_uint8, c_float, c_double, c_uint32, c_bool, c_int

def _find_lib():
    candidates = [
        os.path.join(os.path.dirname(__file__), "..", "..", "build", "librobot_comm_c.so"),
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
        ("fire",                c_uint8),
        ("pitch_target_angle",  c_float),
        ("yaw_torque_only_mode", c_uint8),
        ("yaw_target_angle",    c_double),
        ("yaw_target_velocity", c_float),
        ("yaw_torque",          c_float),
        ("crc8",                c_uint8),
    ]
    def __init__(self, auto_aim_enable=0, pitch_target_angle=0.0,
                 yaw_torque=0.0, fire=0,
                 yaw_torque_only_mode=0, yaw_target_angle=0.0, yaw_target_velocity=0.0):
        super().__init__()
        self.frame_header1 = 0x42; self.frame_header2 = 0x52
        self.protocol_version = 0x02; self.data_size = 23
        self.auto_aim_enable = auto_aim_enable
        self.fire = fire
        self.pitch_target_angle = pitch_target_angle
        self.yaw_torque_only_mode = yaw_torque_only_mode
        self.yaw_target_angle = yaw_target_angle
        self.yaw_target_velocity = yaw_target_velocity
        self.yaw_torque = yaw_torque
        self.crc8 = 0


class McuReceivePacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1",     c_uint8), ("frame_header2",     c_uint8),
        ("protocol_version",  c_uint8), ("data_size",         c_uint8),
        ("bullet_velocity",   c_float), ("pitch_angle",       c_float),
        ("yaw_angle",         c_double),("yaw_omega",         c_float),
        ("chassis_imu_yaw",   c_float), ("chassis_imu_omega", c_float),
        ("mark",              c_uint8), ("color",             c_uint8),
        ("auto_aim_switch",   c_uint8), ("yaw_temperature",   c_uint8), ("crc8",              c_uint8),
    ]


class ImuSendPacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1", c_uint8), ("frame_header2", c_uint8),
        ("frame_header3", c_uint8), ("data_size",     c_uint8),
        ("crc32",         c_uint32),
    ]
    def __init__(self):
        super().__init__()
        self.frame_header1 = 0xA7; self.frame_header2 = 0xB6
        self.frame_header3 = 0xC5; self.data_size = 0; self.crc32 = 0


class ImuReceivePacket(Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_header1", c_uint8), ("frame_header2", c_uint8),
        ("frame_header3", c_uint8), ("data_size",     c_uint8),
        ("gx", c_float), ("gy", c_float), ("gz", c_float),
        ("ax", c_float), ("ay", c_float), ("az", c_float),
        ("euler_yaw", c_double), ("euler_pitch", c_double), ("euler_roll", c_double),
        ("dt_one_tenth_ms", c_uint32), ("crc32", c_uint32),
    ]


class RobotLatestData(Structure):
    _pack_ = 1
    _fields_ = [
        ("imu_valid",  c_bool), ("imu_packet", ImuReceivePacket),
        ("mcu_valid",  c_bool), ("mcu_packet", McuReceivePacket),
    ]


class RobotStrictPose(Structure):
    """严格反解数据包（独立输出）：所有角度 wrap 到 (-π,π]；始终有效，
    缺失数据以 0 参与；R_imu = R_chassis·Rz(yaw_pos)·Rx(pitch_angle) 恒成立"""
    _fields_ = [
        ("imu_euler_yaw",    c_double),   # 反解输入：IMU 欧拉角（始终为 imu 传来数据）
        ("imu_euler_pitch",  c_double),
        ("imu_euler_roll",   c_double),
        ("yaw_pos",          c_double),   # 反解输入：yaw 关节位置（wrap 后）
        ("pitch_angle",      c_double),   # 反解输入：pitch 关节角（wrap 后）
        ("chassis_yaw",      c_double),   # 严格反解底盘欧拉角（wrap 后）
        ("chassis_pitch",    c_double),
        ("chassis_roll",     c_double),
    ]


class RobotFusedData(Structure):
    """融合输出：高频 yaw 位置/速度 + 底盘 world 系姿态"""
    _fields_ = [
        ("valid",              c_bool),
        ("yaw_pos",            c_double),   # yaw 关节解卷绕位置 (rad, 多圈)
        ("yaw_rate",           c_double),   # yaw 关节速度 (rad/s)
        ("chassis_yaw",        c_double),   # 底盘 world 系 yaw（解卷绕）
        ("chassis_pitch",      c_double),   # 底盘 world 系 pitch
        ("chassis_roll",       c_double),   # 底盘 world 系 roll
        ("imu_yaw_unwrapped",  c_double),   # IMU euler yaw 解卷绕
    ]


class YawMpcStepResult(Structure):
    """yaw MPC 一步结果（返回将要发送的值，不发送）"""
    _fields_ = [
        ("yaw_target_angle",    c_double),   # 预测位置 → yaw_target_angle (rad)
        ("yaw_target_velocity", c_double),   # 预测速度 → yaw_target_velocity (rad/s)
        ("yaw_torque",          c_double),   # 控制力矩 → yaw_torque (N·m)
        ("delayed_target",      c_double),   # 当前参考（延迟 dt*N 步的目标）
    ]


class McuMpcState(Structure):
    """实车 MCU 控制封装的最新 mpc 结果（后台线程更新）"""
    _fields_ = [
        ("yaw_target_angle",    c_double),
        ("yaw_target_velocity", c_double),
        ("yaw_torque",          c_double),
        ("delayed_target",      c_double),
    ]

# ── 函数签名 ──
_lib.robot_comm_create.restype             = ctypes.c_void_p
_lib.robot_comm_destroy.argtypes           = [ctypes.c_void_p]
_lib.robot_comm_get_latest_data.argtypes   = [ctypes.c_void_p]
_lib.robot_comm_get_latest_data.restype    = RobotLatestData
_lib.robot_comm_get_fused_data.argtypes    = [ctypes.c_void_p]
_lib.robot_comm_get_fused_data.restype     = RobotFusedData
_lib.robot_comm_get_strict_pose.argtypes   = [ctypes.c_void_p]
_lib.robot_comm_get_strict_pose.restype    = RobotStrictPose
_lib.robot_comm_send_to_mcu.argtypes       = [ctypes.c_void_p, ctypes.POINTER(McuSendPacket)]
_lib.robot_comm_send_to_mcu.restype        = c_bool
_lib.robot_comm_send_to_imu.argtypes       = [ctypes.c_void_p, ctypes.POINTER(ImuSendPacket)]
_lib.robot_comm_send_to_imu.restype        = c_bool
_lib.robot_comm_stop.argtypes              = [ctypes.c_void_p]

_lib.yaw_mpc_create.argtypes = [
    ctypes.c_void_p,                       # RobotCommHandle*
    c_double,                              # dt_control
    c_int,                                 # N
    c_double, c_double, c_double, c_double,   # J, tau_c, b, tau_d
    c_double, c_double,                    # max_torque, max_torque_rate
    c_double, c_double, c_double,          # Q, R, Rd
    c_int,                                 # max_iter
]
_lib.yaw_mpc_create.restype = ctypes.c_void_p
_lib.yaw_mpc_destroy.argtypes = [ctypes.c_void_p]
_lib.yaw_mpc_step.argtypes = [ctypes.c_void_p, c_double]
_lib.yaw_mpc_step.restype = YawMpcStepResult

_lib.mcu_mpc_create.argtypes = _lib.yaw_mpc_create.argtypes
_lib.mcu_mpc_create.restype = ctypes.c_void_p
_lib.mcu_mpc_destroy.argtypes = [ctypes.c_void_p]
_lib.mcu_mpc_step.argtypes = [
    ctypes.c_void_p,                       # handle
    c_uint8, c_uint8,                      # auto_aim_enable, yaw_torque_only_mode
    c_double,                              # target_yaw
    c_float, c_uint8,                      # pitch_target_angle, fire
]
_lib.mcu_mpc_get_state.argtypes = [ctypes.c_void_p]
_lib.mcu_mpc_get_state.restype = McuMpcState


class YawMpcController:
    """yaw MPC 求解器（C++ 实现：参考序列 + 求解；返回发送值，不发送）"""

    def __init__(self, comm_handle, dt_control, N,
                 J, tau_c, b, tau_d=0.0,
                 max_torque=1.0, max_torque_rate=10.0,
                 Q=5.0, R=0.01, Rd=0.1, max_iter=30):
        self._handle = _lib.yaw_mpc_create(
            comm_handle, float(dt_control), int(N),
            J, tau_c, b, tau_d,
            max_torque, max_torque_rate,
            Q, R, Rd, int(max_iter),
        )
        if not self._handle:
            raise RuntimeError("yaw_mpc_create() returned NULL")

    def step(self, target_yaw) -> YawMpcStepResult:
        """传入目标位置，返回 (yaw_target_angle, yaw_target_velocity, yaw_torque, delayed_target)。"""
        return _lib.yaw_mpc_step(self._handle, float(target_yaw))

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.yaw_mpc_destroy(self._handle)
            self._handle = None


class McuMpcController:
    """实车 MCU 控制封装（C++：设置维护 + 后台 100Hz 发送线程）"""

    def __init__(self, comm_handle, dt_control, N,
                 J, tau_c, b, tau_d=0.0,
                 max_torque=1.0, max_torque_rate=10.0,
                 Q=5.0, R=0.01, Rd=0.1, max_iter=30):
        self._handle = _lib.mcu_mpc_create(
            comm_handle, float(dt_control), int(N),
            J, tau_c, b, tau_d,
            max_torque, max_torque_rate,
            Q, R, Rd, int(max_iter),
        )
        if not self._handle:
            raise RuntimeError("mcu_mpc_create() returned NULL")

    def set(self, auto_aim_enable=1, yaw_torque_only_mode=0, target_yaw=0.0,
            pitch_target_angle=0.0, fire=0):
        """设置发送参数 + mpc 目标（顺序：auto_aim_enable, yaw_torque_only_mode,
        target_yaw, pitch_target_angle, fire）。target_yaw 由 C++ 自动转换到与
        imu_yaw_unwrapped 同一圈内；后台线程固定 100Hz 求解并发送给 MCU。"""
        _lib.mcu_mpc_step(self._handle,
                          int(auto_aim_enable), int(yaw_torque_only_mode),
                          float(target_yaw), float(pitch_target_angle), int(fire))

    def get_state(self) -> McuMpcState:
        """最新 mpc 结果（后台线程更新，供显示）"""
        return _lib.mcu_mpc_get_state(self._handle)

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.mcu_mpc_destroy(self._handle)
            self._handle = None


class RobotCommunication:
    """RobotCommunication 的 Python 接口"""

    def __init__(self):
        self._handle = _lib.robot_comm_create()
        if not self._handle:
            raise RuntimeError("robot_comm_create() returned NULL")

    def get_latest_data(self) -> RobotLatestData:
        return _lib.robot_comm_get_latest_data(self._handle)

    def get_fused_data(self) -> RobotFusedData:
        return _lib.robot_comm_get_fused_data(self._handle)

    def get_strict_pose(self) -> RobotStrictPose:
        """严格反解数据包（独立输出；始终有效，角度 wrap 到 (-π,π]）"""
        return _lib.robot_comm_get_strict_pose(self._handle)

    def send_to_mcu(self, packet: McuSendPacket) -> bool:
        return _lib.robot_comm_send_to_mcu(self._handle, ctypes.byref(packet))

    def send_to_imu(self, packet: ImuSendPacket) -> bool:
        return _lib.robot_comm_send_to_imu(self._handle, ctypes.byref(packet))

    def create_mpc(self, **kwargs) -> YawMpcController:
        """创建 yaw MPC 求解器（参数见 YawMpcController；返回发送值，不发送）"""
        return YawMpcController(self._handle, **kwargs)

    def create_mcu_mpc(self, **kwargs) -> McuMpcController:
        """创建实车 MCU 控制封装（后台 100Hz 自动发送，参数见 McuMpcController）"""
        return McuMpcController(self._handle, **kwargs)

    def stop(self):
        _lib.robot_comm_stop(self._handle)

    def close(self):
        if self._handle:
            _lib.robot_comm_destroy(self._handle)
            self._handle = None

    def __enter__(self): return self
    def __exit__(self, *a): self.stop(); self.close()
    def __del__(self): self.close()

# ── 启动时验证 ctypes 布局 ──
assert ctypes.sizeof(McuSendPacket) == 28, f"McuSendPacket size mismatch: {ctypes.sizeof(McuSendPacket)}"
assert ctypes.sizeof(McuReceivePacket) == 37, f"McuReceivePacket size mismatch: {ctypes.sizeof(McuReceivePacket)}"
assert ctypes.sizeof(ImuReceivePacket) == 60, f"ImuReceivePacket size mismatch: {ctypes.sizeof(ImuReceivePacket)}"
