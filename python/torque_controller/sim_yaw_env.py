"""sim_yaw_env.py — yaw 轴动力学仿真环境（与 MPC 内部模型一致）

模型: J * dω/dt = τ - τ_c * tanh(λ*ω) - b * ω + τ_d
欧拉显式积分，子步长 dt_sim（与 MPC 内部预测的 dt_sim 一致）。

供 pygame_control_mpc.py 的 --sim 仿真模式与测试脚本复用：
在模型匹配的前提下，仿真行为即代表实际系统的行为。
"""

import math
import random


class SimYawEnv:
    """yaw 轴动力学仿真环境，多圈连续角度（与 MPC 语义一致，不 wrap）。"""

    def __init__(self, J, tau_c, b, tau_d=0.0, dt_sim=0.002,
                 lambda_omega=100.0, theta0=0.0, omega0=0.0, noise_std=0.0):
        self.J = J
        self.tau_c = tau_c
        self.b = b
        self.tau_d = tau_d
        self.dt_sim = dt_sim
        self.lambda_omega = lambda_omega
        self.noise_std = noise_std
        self.theta = theta0
        self.omega = omega0

    def reset(self, theta=0.0, omega=0.0):
        self.theta = theta
        self.omega = omega
        return self.theta, self.omega

    @property
    def state(self):
        return self.theta, self.omega

    def friction_torque(self, omega):
        soft_sign = math.tanh(self.lambda_omega * omega)
        return -soft_sign * self.tau_c - self.b * omega

    def step(self, tau, dt):
        """应用力矩 tau 持续 dt 秒，返回 (theta, omega)。

        dt 应为 dt_sim 的整数倍；不足一个子步按一个子步处理。
        """
        n = max(1, int(round(dt / self.dt_sim)))
        for _ in range(n):
            alpha = (tau + self.friction_torque(self.omega) + self.tau_d) / self.J
            self.omega += alpha * self.dt_sim
            self.theta += self.omega * self.dt_sim
        if self.noise_std > 0.0:
            self.theta += random.gauss(0.0, self.noise_std)
        return self.theta, self.omega
