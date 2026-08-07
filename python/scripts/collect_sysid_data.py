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

# ============================================================================
# 参数
# ============================================================================
DT             = 0.01
DT_NS          = int(DT * 1e9)
SAMPLE_LEN     = 300
HOLD_TIME      = 2.0
ZERO_TIME      = 1.0
SEED = 42
np.random.seed(SEED)

PID_KP, PID_KI, PID_KD = 2.0, 0.1, 0.2
PID_OUT_MIN, PID_OUT_MAX = -1.0, 1.0
TWO_PI = 2.0 * math.pi

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

    # 5. remainder 到 [-pi, pi]
    seq = np.remainder(seq + math.pi, TWO_PI) - math.pi

    return seq.astype(np.float64)

def busy_wait_until(target_ns):
    while time.perf_counter_ns() < target_ns:
        pass



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
    sample_idx = 0

    def send_zero():
        pkt = McuSendPacket(auto_aim_enable=1, pitch_target_angle=0.0, yaw_torque=0.0, fire=0)
        robot.send_to_mcu(pkt)

    try:
        while True:
            targets = random_target_sequence(all_targets)
            first_target = float(targets[0])
            print(f"\n=== Sample {sample_idx} === first_target={first_target:.3f} rad")

            # ── 预置：PID 跟踪 first_target HOLD_TIME 秒 ──
            print(f"  Settling at target={first_target:.3f} for {HOLD_TIME}s...")
            pid.reset()
            t0 = time.perf_counter_ns()
            while time.perf_counter_ns() - t0 < HOLD_TIME * 1e9:
                data = robot.get_latest_data()
                if data.imu_valid:
                    obs_yaw = float(data.imu_packet.euler_yaw)
                    err = math.remainder(first_target - obs_yaw, TWO_PI)
                    torque = pid.update(err, DT)
                else:
                    torque = 0.0
                pkt = McuSendPacket(auto_aim_enable=1, pitch_target_angle=0.0,
                                    yaw_torque=torque, fire=0)
                robot.send_to_mcu(pkt)
                time.sleep(DT)

            # ── 零力矩 ZERO_TIME 秒 ──
            print(f"  Zero torque for {ZERO_TIME}s...")
            t0 = time.perf_counter_ns()
            while time.perf_counter_ns() - t0 < ZERO_TIME * 1e9:
                send_zero()
                time.sleep(DT)

            # ── 采样 3s @ 100Hz 精确时序 ──
            print(f"  Sampling {SAMPLE_LEN} steps...")
            torque_log, yaw_log, gz_log = [], [], []
            sample_start_ns = time.perf_counter_ns()
            pid.reset()

            for step in range(SAMPLE_LEN):
                target_ns = sample_start_ns + step * DT_NS
                busy_wait_until(target_ns)

                data = robot.get_latest_data()
                if data.imu_valid:
                    obs_yaw = float(data.imu_packet.euler_yaw)
                    obs_gz  = float(data.imu_packet.gz)
                    err = math.remainder(float(targets[step]) - obs_yaw, TWO_PI)
                    torque = pid.update(err, DT)
                else:
                    obs_yaw = 0.0; obs_gz = 0.0; torque = 0.0

                pkt = McuSendPacket(auto_aim_enable=1, pitch_target_angle=0.0,
                                    yaw_torque=torque, fire=0)
                robot.send_to_mcu(pkt)

                torque_log.append(torque)
                yaw_log.append(obs_yaw)
                gz_log.append(obs_gz)

            save_path = os.path.join(SAVE_DIR, f"sample_{sample_idx:04d}.npz")
            np.savez(save_path,
                     target=targets,
                     torque=np.array(torque_log, dtype=np.float32),
                     yaw=np.array(yaw_log, dtype=np.float64),
                     gz=np.array(gz_log, dtype=np.float64))
            print(f"  Saved: {save_path}")
            sample_idx += 1

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        robot.stop()
        robot.close()
        print("Done.")

if __name__ == "__main__":
    main()
