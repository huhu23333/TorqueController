"""mpc_wrapper.py — MPC 求解器 C++ 库 (libmpc_controller.so) 的 ctypes 接口"""

import ctypes
import os
from ctypes import Structure, c_double, c_int, c_void_p, POINTER


def _find_lib():
    candidates = [
        os.path.join(os.path.dirname(__file__), "..", "..", "build", "libmpc_controller.so"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return "libmpc_controller.so"


class MpcControl(Structure):
    _fields_ = [
        ("torque", c_double),
        ("theta",  c_double),
        ("omega",  c_double),
    ]


class MPCController:
    """MPC 求解器 Python 接口

    模型: J * dω/dt = τ - τ_c*sign(ω) - b*ω + τ_d
    限制: 最高速度（力矩投影硬保证）/ 最大力矩 / 最大力矩变化率
    """

    def __init__(self, dt_control, dt_sim,
                 J, tau_c, b, tau_d=0.0,
                 max_omega=30.0, max_torque=1.0, max_torque_rate=10.0,
                 N=20, Q=5.0, R=0.01, Rd=0.1, max_iter=30, lib_path=None):
        self.dt_control = dt_control
        self.dt_sim = dt_sim
        self.N = N
        self.max_omega = max_omega
        self.max_torque = max_torque
        self.max_torque_rate = max_torque_rate

        if lib_path is None:
            lib_path = _find_lib()
        self._lib = ctypes.CDLL(str(lib_path))

        self._lib.mpc_create.argtypes = [
            c_double, c_double,                       # dt_control, dt_sim
            c_double, c_double, c_double, c_double,   # J, tau_c, b, tau_d
            c_double, c_double, c_double,             # max_omega, max_torque, max_torque_rate
            c_int,                                    # N
            c_double, c_double, c_double,             # Q, R, Rd
            c_int,                                    # max_iter
        ]
        self._lib.mpc_create.restype = c_void_p

        self._lib.mpc_destroy.argtypes = [c_void_p]
        self._lib.mpc_destroy.restype = None

        self._lib.mpc_step.argtypes = [
            c_void_p,
            c_double, c_double,           # theta, omega
            POINTER(c_double), c_int,     # theta_ref, ref_len
        ]
        self._lib.mpc_step.restype = MpcControl

        self._handle = self._lib.mpc_create(
            dt_control, dt_sim,
            J, tau_c, b, tau_d,
            max_omega, max_torque, max_torque_rate,
            N, Q, R, Rd, max_iter,
        )
        if not self._handle:
            raise RuntimeError("mpc_create() returned NULL")

    def step(self, theta, omega, theta_ref):
        """求解一步 MPC。

        参数:
            theta, omega: 当前状态 (位置 rad, 速度 rad/s，均为多圈连续角度)
            theta_ref:    未来参考轨迹，长度 N（ref[i] 对应预测 theta_pred[i+1]），
                          多圈连续角度，误差直接相减不归一化

        返回:
            (torque, theta_pred, omega_pred) —— 第一步控制力矩 + 预测位置 + 预测速度
        """
        ref = [float(x) for x in theta_ref]
        if len(ref) < self.N:
            raise ValueError(f"theta_ref length must be >= {self.N}, got {len(ref)}")

        arr = (c_double * self.N)(*ref[:self.N])
        out = self._lib.mpc_step(self._handle, float(theta), float(omega), arr, self.N)
        return out.torque, out.theta, out.omega

    def __del__(self):
        if getattr(self, "_handle", None):
            self._lib.mpc_destroy(self._handle)
            self._handle = None
