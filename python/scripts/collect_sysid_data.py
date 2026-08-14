"""
collect_sysid_data.py — 系统辨识数据采集

协议:
  - 从 data/targets/*.npz 加载 target 序列
  - 随机截取 seq_len=300 (3s@100Hz) 的 target 片段
  - 每条采样前：先设 PID 目标为序列首值维持 2s，再设 torque=0 维持 1s
  - 采样 3s：用 time.perf_counter_ns() 精确控制 100Hz，采集 torque/imu_yaw/imu_gz
  - 保存到 data/sysid_samples/
"""

import sys, os, math, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from torque_controller import RobotCommunication, McuSendPacket
from scripts.trajectory_planner import TrajectoryPlanner, StepRefinementWrapper

# ============================================================================
# 参数
# ============================================================================
DT             = 0.01
DT_NS          = int(DT * 1e9)
SAMPLE_LEN     = 300
SEED = 42
np.random.seed(SEED)

# 轨迹规划器参数 (与 trajectory_viz 一致)
MAX_VEL   = 30.0       # rad/s
MAX_ACCEL = 50.0       # rad/s²
MAX_JERK  = 2000.0     # rad/s³
REFINE_N  = 1000       # StepRefinementWrapper 细化系数

PID_KP, PID_KI, PID_KD = 2.0, 0.1, 0.2
PID_OUT_MIN, PID_OUT_MAX = -1.0, 1.0
TWO_PI = 2.0 * math.pi

# 力矩变化率限制
MAX_TORQUE_DELTA = 0.1  # 相邻两步力矩差异不超过此值

# 过热检测
OVERHEAT_ACTION = "wait"   # "exit" 或 "wait"
OVERHEAT_WAIT_S = 600      # 等待秒数（默认 10 分钟）
OVERHEAT_DEG    = 20.0     # 1s 末与 2s 末角度差 < 此值判过热
OVERHEAT_TEMP   = 55       # 温度 ≥ 此值判过热

TARGET_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "data", "targets")
SAVE_DIR   = os.path.join(os.path.dirname(__file__), "..", "..", "data", "sysid_samples")

# ============================================================================
# PID
# ============================================================================
class PidController:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0; self.prev_error = 0.0

    def update(self, error, dt):
        deriv = (error - self.prev_error) / dt if dt > 1e-6 else 0.0
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        sat_hi = out > self.out_max; sat_lo = out < self.out_min
        if sat_hi: out = self.out_max
        if sat_lo: out = self.out_min
        do_int = True
        if sat_hi and error > 0: do_int = False
        if sat_lo and error < 0: do_int = False
        if do_int: self.integral += error * dt
        return out

    def reset(self): self.integral = 0.0; self.prev_error = 0.0

# ============================================================================
# 目标序列加载
# ============================================================================
def load_all_targets():
    files = sorted(f for f in os.listdir(TARGET_DIR) if f.endswith('.npz'))
    all_targets = []
    for f in files:
        data = np.load(os.path.join(TARGET_DIR, f))
        all_targets.append(data['target'].astype(np.float64))
    print(f"Loaded {len(files)} files, total target points: {sum(len(t) for t in all_targets)}")
    return all_targets

def random_target_sequence(all_targets):
    """随机选一条，截取 SAMPLE_LEN 点，连续化 + 随机缩放/偏置"""
    # 1. 随机选文件、随机截取
    arr = all_targets[np.random.randint(0, len(all_targets))]
    if len(arr) <= SAMPLE_LEN:
        seq = arr.copy()
    else:
        start = np.random.randint(0, len(arr) - SAMPLE_LEN)
        seq = arr[start:start + SAMPLE_LEN].copy()

    # 2. 连续化：相邻点跳变 > pi 时，后续点 ±2π 使 delta ≤ pi
    seq = seq.copy()
    for i in range(1, len(seq)):
        delta = seq[i] - seq[i - 1]
        if delta > math.pi:
            seq[i] -= TWO_PI
        elif delta < -math.pi:
            seq[i] += TWO_PI

    # 3. 随机缩放 [0.5, 1.0]
    scale = np.random.uniform(0.5, 1.0)
    seq *= scale

    # 4. 随机偏置 [-pi, pi]
    bias = np.random.uniform(-math.pi, math.pi)
    seq += bias

    # 不在此处 remainder，留给平滑后再做

    return seq.astype(np.float64)

def busy_wait_until(target_ns):
    while time.perf_counter_ns() < target_ns:
        pass

def compute_smooth_targets(raw_targets, dt, planner, refined_planner):
    """将 raw_targets 通过 TrajectoryPlanner+StepRefinementWrapper 平滑化
    返回平滑后的位置轨迹 (np.ndarray, shape=(len(raw_targets),))"""
    n = len(raw_targets)
    smooth = np.zeros(n, dtype=np.float64)
    pos = float(raw_targets[0])
    vel = 0.0
    acc = 0.0
    for i in range(n):
        pos, vel, acc, _ = refined_planner.step(
            float(raw_targets[i]), pos, vel, acc, dt
        )
        smooth[i] = pos
    return smooth

def limit_torque(torque, last_torque):
    """限制力矩变化率不超过 MAX_TORQUE_DELTA"""
    delta = torque - last_torque
    if delta > MAX_TORQUE_DELTA:
        return last_torque + MAX_TORQUE_DELTA
    elif delta < -MAX_TORQUE_DELTA:
        return last_torque - MAX_TORQUE_DELTA
    return torque



# ============================================================================
# 主流程
# ============================================================================
def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    all_targets = load_all_targets()

    print("Connecting to robot...")
    robot = RobotCommunication()
    print("Waiting for data...")
    while True:
        data = robot.get_latest_data()
        if data.imu_valid and data.mcu_valid:
            break
        time.sleep(0.01)
    print("Ready.")

    pid = PidController(PID_KP, PID_KI, PID_KD, PID_OUT_MIN, PID_OUT_MAX)
    last_saved = None
    overheat_deg = math.radians(OVERHEAT_DEG)

    # 轨迹规划器 (参数与 trajectory_viz 一致)
    planner = TrajectoryPlanner(max_velocity=MAX_VEL, max_acceleration=MAX_ACCEL, max_jerk=MAX_JERK)
    refined_planner = StepRefinementWrapper(planner.step, REFINE_N)

    # 力矩变化率限制用，跨 send_to_mcu 调用共享
    last_torque = [0.0]  # 用列表实现 mutable closure

    def check_temp(data):
        mcu = data.mcu_packet
        return mcu.yaw_temperature if data.mcu_valid else 0

    def send_zero():
        torque = limit_torque(0.0, last_torque[0])
        pkt = McuSendPacket(auto_aim_enable=1, pitch_target_angle=0.0,
                            yaw_torque_only_mode=1, yaw_target_angle=0.0,
                            yaw_target_velocity=0.0, yaw_torque=torque, fire=0)
        robot.send_to_mcu(pkt)
        last_torque[0] = torque

    def pid_to_target(target, duration):
        pid.reset()
        t0 = time.perf_counter_ns()
        end_yaw = 0.0
        step = 0
        while time.perf_counter_ns() - t0 < duration * 1e9:
            target_ns = t0 + step * DT_NS
            busy_wait_until(target_ns)
            step += 1
            data = robot.get_latest_data()
            if check_temp(data) >= OVERHEAT_TEMP:
                return end_yaw, True
            obs_yaw = 0.0
            if data.imu_valid:
                obs_yaw = float(data.imu_packet.euler_yaw)
                err = math.remainder(target - obs_yaw, TWO_PI)
                torque = pid.update(err, DT)
            else:
                torque = 0.0
            torque = limit_torque(torque, last_torque[0])
            pkt = McuSendPacket(auto_aim_enable=1, pitch_target_angle=0.0,
                                yaw_torque_only_mode=1, yaw_target_angle=0.0,
                                yaw_target_velocity=0.0, yaw_torque=torque, fire=0)
            robot.send_to_mcu(pkt)
            last_torque[0] = torque
            end_yaw = obs_yaw
        return end_yaw, False

    def handle_overheat(reason):
        if last_saved and os.path.exists(last_saved):
            os.remove(last_saved)
            print(f"  Removed previous sample: {last_saved}")
        print(f"  OVERHEAT ({reason}), cooling down {OVERHEAT_WAIT_S}s...")
        total_ns = int(OVERHEAT_WAIT_S * 1e9)
        t0 = time.perf_counter_ns()
        step = 0
        while time.perf_counter_ns() - t0 < total_ns:
            target_ns = t0 + step * DT_NS
            busy_wait_until(target_ns)
            step += 1
            send_zero()
            if step % 100 == 0:
                remaining = int(OVERHEAT_WAIT_S) - (step // 100)
                temp = check_temp(robot.get_latest_data())
                print(f"  Cooling... temp={temp}°C  remaining={remaining}s  reason={reason}")
        if OVERHEAT_ACTION == "exit":
            print("  Exiting.")
            sys.exit(1)
        print("  Resuming...")

    try:
        while True:
            targets_raw = random_target_sequence(all_targets)

            # ── 使用 TrajectoryPlanner + StepRefinementWrapper 平滑原始目标轨迹 ──
            targets_smooth = compute_smooth_targets(targets_raw, DT, planner, refined_planner)
            # 平滑后再 remainder 到 [-pi, pi]
            targets_smooth = np.remainder(targets_smooth, TWO_PI)
            first_target = float(targets_smooth[0])
            ts = int(time.time())
            print(f"\n=== Sample {ts} === first_target={first_target:.3f} rad")
            temp = check_temp(robot.get_latest_data())
            print(f"  Current temp: {temp}°C")

            # ── 2s: PID 到 first_target + 30° ──
            target_plus = first_target + math.radians(30)
            print(f"  PID to {math.degrees(target_plus):.1f}° for 2s...")
            yaw1, oh = pid_to_target(target_plus, 2.0)
            if oh: handle_overheat(f"temp>={OVERHEAT_TEMP}"); continue

            # ── 2s: PID 到 first_target ──
            print(f"  PID to {math.degrees(first_target):.1f}° for 2s...")
            yaw2, oh = pid_to_target(first_target, 2.0)
            if oh: handle_overheat(f"temp>={OVERHEAT_TEMP}"); continue

            # ── 过热检测（动作幅度） ──
            diff = abs(math.remainder(yaw1 - yaw2, TWO_PI))
            print(f"  yaw1={math.degrees(yaw1):.1f}° yaw2={math.degrees(yaw2):.1f}° diff={math.degrees(diff):.1f}°")
            if diff < overheat_deg:
                handle_overheat(f"motion<{OVERHEAT_DEG}°")
                continue

            # ── 1s: 零力矩 ──
            print(f"  Zero torque for 1s...")
            t0 = time.perf_counter_ns()
            step = 0
            while time.perf_counter_ns() - t0 < 1e9:
                target_ns = t0 + step * DT_NS
                busy_wait_until(target_ns)
                step += 1
                send_zero()

            # ── 采样 3s ──
            print(f"  Sampling {SAMPLE_LEN} steps...")
            torque_log, yaw_log, gz_log = [], [], []
            sample_start_ns = time.perf_counter_ns()
            pid.reset()

            for step in range(SAMPLE_LEN):
                target_ns = sample_start_ns + step * DT_NS
                busy_wait_until(target_ns)
                data = robot.get_latest_data()
                if check_temp(data) >= OVERHEAT_TEMP:
                    handle_overheat(f"temp>={OVERHEAT_TEMP}")
                    break
                if data.imu_valid:
                    obs_yaw = float(data.imu_packet.euler_yaw)
                    obs_gz  = float(data.imu_packet.gz)
                    err = math.remainder(float(targets_smooth[step]) - obs_yaw, TWO_PI)
                    torque = pid.update(err, DT)
                else:
                    obs_yaw = 0.0; obs_gz = 0.0; torque = 0.0
                torque = limit_torque(torque, last_torque[0])
                pkt = McuSendPacket(auto_aim_enable=1, pitch_target_angle=0.0,
                                    yaw_torque_only_mode=1, yaw_target_angle=0.0,
                                    yaw_target_velocity=0.0, yaw_torque=torque, fire=0)
                robot.send_to_mcu(pkt)
                last_torque[0] = torque
                torque_log.append(torque)
                yaw_log.append(obs_yaw)
                gz_log.append(obs_gz)

            save_path = os.path.join(SAVE_DIR, f"sample_{ts}.npz")
            np.savez(save_path,
                     target_raw=targets_raw,
                     target=targets_smooth,
                     torque=np.array(torque_log, dtype=np.float32),
                     yaw=np.array(yaw_log, dtype=np.float64),
                     gz=np.array(gz_log, dtype=np.float64))
            print(f"  Saved: {save_path}")
            last_saved = save_path

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        robot.stop()
        robot.close()
        print("Done.")

if __name__ == "__main__":
    main()
