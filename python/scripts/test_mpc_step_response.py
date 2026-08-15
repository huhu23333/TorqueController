"""
test_mpc_step_response.py — MPC 阶跃响应测试

验证在力矩变化率限幅（max_torque_rate）下，MPC 输出平滑控制量的同时
尽可能快速收敛到阶跃目标：

  1. 相邻步力矩差 |Δτ| ≤ rate_step（限幅生效，控制量无突变）
  2. 收敛时间（settle time）尽量短
  3. 过冲可控、力矩方向翻转次数少（无振荡）
  4. 峰值速度（信息性报告，MPC 当前不施加速度限制）

闭环推进使用 SimYawEnv（与 MPC 内部模型一致），多圈连续角度；
参考轨迹长度 N（ref[i] 对应预测 theta_pred[i+1]），误差直接相减。

运行:
    PYTHONPATH=python python3 python/scripts/test_mpc_step_response.py
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from torque_controller.mpc_wrapper import MPCController
from torque_controller.sim_yaw_env import SimYawEnv

# ================== 参数（与 pygame_control_mpc.py 一致）==================
J, TAU_C, B, TAU_D = 0.016541, 0.097297, 0.0321, 0.0
MAX_TORQUE, MAX_TORQUE_RATE = 1.0, 10.0
N = 20
DT = 0.01
DT_SIM = 0.002
RATE_STEP = MAX_TORQUE_RATE * DT          # 0.1 N·m/步
SETTLE_TOL = 0.05                         # 收敛判据 (rad)
HOLD_STEPS = 50                           # 阶跃前稳定时间 (0.5s)
MAX_STEPS = 800


def make_mpc():
    return MPCController(dt_control=DT, dt_sim=DT_SIM,
                         J=J, tau_c=TAU_C, b=B, tau_d=TAU_D,
                         max_torque=MAX_TORQUE, max_torque_rate=MAX_TORQUE_RATE,
                         N=N, Q=5.0, R=0.01, Rd=0.1, max_iter=30)


def step_response(step_target, max_steps=MAX_STEPS):
    """从 0 稳定 0.5s 后阶跃到 step_target，返回指标 dict。"""
    mpc = make_mpc()
    env = SimYawEnv(J, TAU_C, B, TAU_D, dt_sim=DT_SIM)
    env.reset(0.0, 0.0)
    theta, omega = env.state

    # 阶跃前稳定在 0
    tau = 0.0
    for _ in range(HOLD_STEPS):
        tau, _, _ = mpc.step(theta, omega, [0.0] * N)
        theta, omega = env.step(tau, DT)

    # 阶跃响应
    settle_time = None
    peak_omega = 0.0
    max_dtau = 0.0
    rms_dtau = 0.0
    flips = 0
    overshoot = 0.0
    tau_prev = tau
    prev_dir = 0
    n_dtau = 0
    for i in range(max_steps):
        tau, _, _ = mpc.step(theta, omega, [step_target] * N)

        d = tau - tau_prev
        ad = abs(d)
        max_dtau = max(max_dtau, ad)
        rms_dtau += ad * ad
        n_dtau += 1
        if ad > 1e-6:
            cur_dir = 1 if d > 0 else -1
            if prev_dir != 0 and cur_dir != prev_dir:
                flips += 1
            prev_dir = cur_dir

        theta, omega = env.step(tau, DT)
        peak_omega = max(peak_omega, abs(omega))
        overshoot = max(overshoot, abs(theta - step_target) - abs(step_target))  # 越界量
        if settle_time is None and abs(step_target - theta) < SETTLE_TOL:
            settle_time = (i + 1) * DT
        tau_prev = tau

    return {
        "target": step_target,
        "settle_time": settle_time,
        "peak_omega": peak_omega,
        "max_dtau": max_dtau,
        "rms_dtau": math.sqrt(rms_dtau / max(1, n_dtau)),
        "flips": flips,
        "overshoot": max(0.0, overshoot),
        "final_theta": theta,
    }


def main():
    print("=" * 62)
    print("MPC 阶跃响应测试  (rate_step = {:.3f} N·m/步, settle_tol = {:.2f} rad)".format(RATE_STEP, SETTLE_TOL))
    print("=" * 62)
    all_ok = True
    for target in [3.0, -3.0, 1.5]:
        r = step_response(target)
        rate_ok = r["max_dtau"] <= RATE_STEP + 1e-6
        all_ok = all_ok and rate_ok
        settle = f"{r['settle_time']:.3f} s" if r["settle_time"] is not None else "> {:.1f} s".format(MAX_STEPS * DT)
        print(f"\n阶跃 {r['target']:+.3f} rad:")
        print(f"  收敛时间 settle_time = {settle}")
        print(f"  峰值速度 peak|ω|    = {r['peak_omega']:.3f} rad/s (信息性, 当前无速度限制)")
        print(f"  最大力矩变化 max|Δτ| = {r['max_dtau']:.4f} N·m (rate_step {RATE_STEP})  "
              f"[{'PASS' if rate_ok else 'FAIL'}]")
        print(f"  力矩变化 RMS |Δτ|   = {r['rms_dtau']:.4f} N·m")
        print(f"  力矩方向翻转次数    = {r['flips']}  (平滑性参考, 越小越好)")
        print(f"  过冲 overshoot      = {r['overshoot']:.4f} rad")
        print(f"  终点 theta          = {r['final_theta']:.4f} rad")
    print("\n" + "=" * 62)
    print("RESULT:", "ALL PASS" if all_ok else "SOME FAILED")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
