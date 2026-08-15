"""
pygame_control_mpc.py — 使用 pygame 鼠标控制 yaw（MPC）/ pitch 的实车控制程序
                   结构仿照 pygame_control.py（同时获取和绘制 yaw 与 pitch）

- yaw: MPC 力矩控制（闭环完全在 C++：McuMpcController 后台 100Hz 线程求解并发送）。
       目标 yaw 可选用 TrajectoryPlanner + StepRefinementWrapper 平滑（USE_PLANNER）；
       目标延迟 dt*N 步由 C++ 内部维护（delayed_target）。
       目标 yaw 速度 = mpc 解算的 yaw_target_velocity；Torque = mpc 的 yaw_torque。
- pitch: 鼠标竖向控制目标，限定 [-10°, +20°]，经 mcu_mpc.set 设置发送参数。
- 观测: 融合滤波器输出（yaw_pos/yaw_rate/imu_yaw_unwrapped）+ MCU（pitch/temp）
- 时间控制: time.perf_counter_ns() + 忙等待，每帧对齐到 start + frame*DT_S

运行:
    PYTHONPATH=python python3 python/scripts/pygame_control_mpc.py
"""

import sys, os, math, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from torque_controller import RobotCommunication
from scripts.trajectory_planner import TrajectoryPlanner, StepRefinementWrapper

import pygame
from pygame.locals import *
from collections import deque

# ============================================================================
# 配置（与 pygame_control.py 一致）
# ============================================================================
WINDOW_W, WINDOW_H = 1920, 1080
DT_CTRL = 0.01          # MPC 控制周期 (100 Hz，与后台发送线程一致)
DT_S = DT_CTRL
FPS = 100

LINE_LEN = 200
CIRCLE_R = 10
CIRCLE_Y = 200
PITCH_MIN_DEG = -10.0
PITCH_MAX_DEG =  20.0
YAW_SENS   =  0.003     # 鼠标水平每像素 → yaw rad
PITCH_SENS =  0.003     # 鼠标竖向每像素 → pitch rad

# yaw 目标平滑（可选：与 pygame_control.py 相同的 TrajectoryPlanner）
USE_PLANNER = False
MAX_VEL   = 30.0        # rad/s
MAX_ACCEL = 50.0        # rad/s²
MAX_JERK  = 2000.0      # rad/s³
REFINE_N  = 1000

# 目标延迟（C++ 内部维护，延迟 dt*N 步）
DELAY_TIME = 0.2        # 秒
MPC_PRED_N = int(DELAY_TIME / DT_CTRL)

# ================== 辨识参数 (params/1/Identified_parameters.txt) ==================
J       = 0.016541
TAU_C   = 0.097297
B_FRIC  = 0.032100
TAU_D   = 0.0            # tau_d 设为 0

# ================== 约束 ==================
MAX_TORQUE      = 1.0    # 最大力矩 (N·m)
MAX_TORQUE_RATE = 40.0   # 最大力矩变化率 (N·m/s)

# ================== 波形图参数（与 pygame_control.py 一致）==================
WAVE_LEFT   = 80
WAVE_RIGHT  = 1840
WAVE_WIDTH  = WAVE_RIGHT - WAVE_LEFT
HISTORY_S   = 3.0
HISTORY_N   = int(HISTORY_S * FPS)

RED    = (255, 60, 60)
BLUE   = (60, 120, 255)
WHITE  = (240, 240, 240)
BLACK  = (20, 20, 20)
GRAY   = (80, 80, 80)
DGRAY  = (40, 40, 40)
GREEN  = (100, 200, 100)
YELLOW = (220, 220, 60)
ORANGE = (255, 160, 40)
LGRAY  = (65, 65, 65)

TWO_PI = 2.0 * math.pi

# 绝对模式控制框（与 pygame_control.py 一致）
BOX_W, BOX_H = 900, 120
BOX_X = (WINDOW_W - BOX_W) // 2
BOX_Y = 460


def busy_wait_until(target_s):
    """忙等待到绝对时间点"""
    while time.perf_counter_ns() < target_s * 1e9:
        pass


# ============================================================================
# 滚动波形缓冲区（复制自 pygame_control.py）
# ============================================================================
class RingBuffer:
    def __init__(self, capacity, fill=0.0):
        self._buf = deque([float(fill)] * capacity, maxlen=capacity)

    def push(self, value):
        self._buf.append(float(value))

    def data(self):
        return list(self._buf)

    def min_max(self):
        d = self._buf
        if not d:
            return 0.0, 1.0
        mn = min(d)
        mx = max(d)
        if mx - mn < 1e-9:
            return mn - 0.5, mx + 0.5
        margin = (mx - mn) * 0.1
        return mn - margin, mx + margin


# ============================================================================
# 绘图（复制自 pygame_control.py）
# ============================================================================
def draw_angle_view(surf, cx, cy, target_angle, current_angle, title, scale=1.0, rotate_ccw90=False):
    """以 (cx,cy) 为圆心画两条射线表示目标和当前角度"""
    t_ang = target_angle * scale
    c_ang = current_angle * scale
    if rotate_ccw90:
        ex = cx - LINE_LEN * math.sin(t_ang)
        ey = cy - LINE_LEN * math.cos(t_ang)
    else:
        ex = cx + LINE_LEN * math.cos(t_ang)
        ey = cy - LINE_LEN * math.sin(t_ang)
    pygame.draw.line(surf, RED, (cx, cy), (int(ex), int(ey)), 3)
    if rotate_ccw90:
        ex = cx - LINE_LEN * math.sin(c_ang)
        ey = cy - LINE_LEN * math.cos(c_ang)
    else:
        ex = cx + LINE_LEN * math.cos(c_ang)
        ey = cy - LINE_LEN * math.sin(c_ang)
    pygame.draw.line(surf, BLUE, (cx, cy), (int(ex), int(ey)), 3)
    pygame.draw.circle(surf, WHITE, (cx, cy), CIRCLE_R)
    font = pygame.font.SysFont(None, 22)
    s = font.render(title, True, WHITE)
    surf.blit(s, (cx - s.get_width() // 2, cy + LINE_LEN + 10))


def draw_info(surf, fused, data, target_yaw, target_pitch, mpc_state, fps, mode):
    font = pygame.font.SysFont(None, 20)
    y = 5
    def t(txt):
        nonlocal y
        s = font.render(txt, True, WHITE)
        surf.blit(s, (5, y)); y += 22

    mcu = data.mcu_packet
    mode_names = {0: "RELATIVE (click lock)", 1: "ABSOLUTE (box)"}
    t(f"FPS: {fps:.0f}  |  Mode: {mode_names.get(mode, '?')}  |  [TAB:switch ESC:quit R:reset]")
    t(f"Target  Yaw: {target_yaw:7.3f} rad  |  Pitch: {target_pitch:.3f} rad")
    t(f"MPC     Yaw target_angle: {mpc_state.yaw_target_angle:7.3f}  velocity: {mpc_state.yaw_target_velocity:+.3f}  torque: {mpc_state.yaw_torque:+.4f}")
    t(f"Fused   Yaw pos: {fused.yaw_pos:7.3f}  rate: {fused.yaw_rate:+.3f}  imu_yaw: {fused.imu_yaw_unwrapped:7.3f}")
    t(f"Fused   chassis: yaw={fused.chassis_yaw:.3f} pitch={fused.chassis_pitch:.3f} roll={fused.chassis_roll:.3f}")
    t(f"MCU Pitch: {mcu.pitch_angle:.3f}  |  Temp: {mcu.yaw_temperature}\u00b0C  |  fused_valid: {fused.valid}")


def draw_control_box(surf, mx, my):
    """绘制绝对模式控制框和十字准星（复制自 pygame_control.py）"""
    r = pygame.Rect(BOX_X, BOX_Y, BOX_W, BOX_H)
    pygame.draw.rect(surf, GRAY, r, 1)
    cx = BOX_X + BOX_W // 2; cy = BOX_Y + BOX_H // 2
    pygame.draw.line(surf, DGRAY, (BOX_X, cy), (BOX_X + BOX_W, cy), 1)
    pygame.draw.line(surf, DGRAY, (cx, BOX_Y), (cx, BOX_Y + BOX_H), 1)
    font = pygame.font.SysFont(None, 18)
    s = font.render("yaw: L=+pi  R=-pi", True, GRAY)
    surf.blit(s, (BOX_X, BOX_Y - 18))
    s = font.render("pitch: up=+  down=-", True, GRAY)
    surf.blit(s, (BOX_X + BOX_W - s.get_width(), BOX_Y + BOX_H + 2))
    cx_clamp = max(BOX_X, min(BOX_X + BOX_W, mx))
    cy_clamp = max(BOX_Y, min(BOX_Y + BOX_H, my))
    pygame.draw.line(surf, YELLOW, (cx_clamp - 10, cy_clamp), (cx_clamp + 10, cy_clamp), 2)
    pygame.draw.line(surf, YELLOW, (cx_clamp, cy_clamp - 10), (cx_clamp, cy_clamp + 10), 2)


def draw_waveform(surf, x, y, w, h, title, buf, y_label,
                  line_color, y_min=None, y_max=None, extra_traces=None):
    """滚动波形图，支持多条曲线叠加"""
    rect = pygame.Rect(x, y, w, h)
    pygame.draw.rect(surf, DGRAY, rect, 1)
    pygame.draw.rect(surf, BLACK, rect.inflate(-2, -2))

    font = pygame.font.SysFont(None, 16)
    s = font.render(title, True, WHITE)
    surf.blit(s, (x + 4, y + 2))

    legend_x = x + s.get_width() + 10
    traces = [(buf, line_color)] + (extra_traces or [])
    for _, c in traces:
        pygame.draw.rect(surf, c, (legend_x, y + 5, 10, 10))
        legend_x += 14
    s = font.render(y_label, True, GRAY)
    surf.blit(s, (x + w - s.get_width() - 4, y + 2))

    if y_min is not None and y_max is not None:
        y_min, y_max = float(y_min), float(y_max)
    else:
        all_data = []
        for b, _ in traces:
            all_data.extend(b.data())
        if len(all_data) < 2:
            return
        y_min, y_max = min(all_data), max(all_data)
        if y_max - y_min < 1e-9:
            y_min -= 0.5; y_max += 0.5
        else:
            m = (y_max - y_min) * 0.1
            y_min -= m; y_max += m

    for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        gy = int(y + h - 4 - frac * (h - 28))
        pygame.draw.line(surf, LGRAY, (x + 2, gy), (x + w - 2, gy), 1)
        val = y_min + frac * (y_max - y_min)
        sv = font.render(f"{val:.1f}", True, GRAY)
        surf.blit(sv, (x + w - sv.get_width() - 4, gy - sv.get_height()))

    for b, c in traces:
        data = b.data()
        n = len(data)
        if n < 2:
            continue
        points = []
        for i, val in enumerate(data):
            frac_x = i / max(n - 1, 1)
            frac_y = (val - y_min) / max(y_max - y_min, 1e-9)
            px = int(x + 6 + frac_x * (w - 14))
            py = int(y + h - 8 - frac_y * (h - 32))
            points.append((px, py))
        if len(points) >= 2:
            pygame.draw.lines(surf, c, False, points, 2)


# ============================================================================
# 主函数
# ============================================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Yaw (MPC) / Pitch Mouse Control")
    font = pygame.font.Font(None, 26)

    # 通信 + 融合滤波器
    robot = RobotCommunication()
    print("Waiting for fused data (IMU + MCU yaw)...")
    while True:
        fused = robot.get_fused_data()
        if fused.valid:
            break
        time.sleep(0.01)
    print("Fused data ready. Starting control loop.")

    # 实车 MCU 控制封装（C++ 后台 100Hz：参考序列 + MPC 求解 + 发送）
    mcu_mpc = robot.create_mcu_mpc(
        dt_control=DT_CTRL, N=MPC_PRED_N,
        J=J, tau_c=TAU_C, b=B_FRIC, tau_d=TAU_D,
        max_torque=MAX_TORQUE, max_torque_rate=MAX_TORQUE_RATE,
        Q=5.0, R=0.01, Rd=0.1, max_iter=30,
    )

    # 可选：yaw 目标平滑（与 pygame_control.py 相同）
    planner = None
    yaw_pos_p, yaw_vel_p, yaw_acc_p = 0.0, 0.0, 0.0
    if USE_PLANNER:
        planner = TrajectoryPlanner(max_velocity=MAX_VEL, max_acceleration=MAX_ACCEL, max_jerk=MAX_JERK)
        planner = StepRefinementWrapper(planner.step, REFINE_N)
        yaw_pos_p = fused.yaw_pos

    # 目标（相对当前 yaw，连续多圈；延迟与参考序列由 C++ 内部维护）
    target_yaw = fused.yaw_pos
    target_pitch = 0.0
    pitch_min = math.radians(PITCH_MIN_DEG)
    pitch_max = math.radians(PITCH_MAX_DEG)

    # 波形历史缓冲区
    mpc_state = mcu_mpc.get_state()
    hist_delayed_target = RingBuffer(HISTORY_N, fused.imu_yaw_unwrapped)
    hist_cur_yaw        = RingBuffer(HISTORY_N, fused.imu_yaw_unwrapped)
    hist_vel_sent       = RingBuffer(HISTORY_N, 0.0)
    hist_omega          = RingBuffer(HISTORY_N, 0.0)
    hist_torque         = RingBuffer(HISTORY_N, 0.0)

    VEL_Y_RANGE   = (-MAX_VEL * 1.2, MAX_VEL * 1.2)
    TORQUE_Y_RANGE = (-2.0, 2.0)

    running = True
    mode = 0                     # 0=相对(点击锁定持续捕获)  1=绝对(框)
    total_time = 0.0
    loop_start_s = time.perf_counter_ns() * 1e-9
    already_exceeded_time = 0.0
    frame = 0

    while running:
        frame += 1
        total_time = time.perf_counter_ns() * 1e-9 - loop_start_s
        delay_target_s = loop_start_s + frame * DT_S + already_exceeded_time
        if time.perf_counter_ns() * 1e-9 < delay_target_s:
            busy_wait_until(delay_target_s)
        else:
            already_exceeded_time = time.perf_counter_ns() * 1e-9 - (loop_start_s + frame * DT_S)

        mx, my = pygame.mouse.get_pos()

        # ── 事件处理（与 pygame_control.py 一致：点击锁定持续捕获，TAB 切换模式）──
        for evt in pygame.event.get():
            if evt.type == QUIT:
                running = False
            elif evt.type == KEYDOWN:
                if evt.key == K_ESCAPE:
                    running = False
                elif evt.key == K_TAB:
                    if mode == 0:
                        mode = 1
                        pygame.mouse.set_visible(True)
                        pygame.event.set_grab(False)
                    else:
                        mode = 0
                elif evt.key == K_r:
                    target_yaw = fused.yaw_pos if fused.valid else target_yaw
                    target_pitch = 0.0
            elif evt.type == MOUSEBUTTONDOWN and mode == 0:
                # 点击锁定鼠标，之后持续捕获相对移动（无需按住）
                pygame.mouse.set_visible(False)
                pygame.event.set_grab(True)

        # ── 目标角度（与 pygame_control.py 一致）──
        if mode == 0:
            if pygame.event.get_grab():
                dx, dy = pygame.mouse.get_rel()
                target_yaw   -= float(dx) * YAW_SENS
                target_pitch -= float(dy) * PITCH_SENS
        else:
            fx = (mx - BOX_X) / BOX_W
            fy = (my - BOX_Y) / BOX_H
            fx = max(0.0, min(1.0, fx))
            fy = max(0.0, min(1.0, fy))
            target_yaw   = (1.0 - fx) * TWO_PI - math.pi
            target_pitch = (1.0 - fy) * (pitch_max - pitch_min) + pitch_min

        # pitch 限位
        if target_pitch > pitch_max: target_pitch = pitch_max
        if target_pitch < pitch_min: target_pitch = pitch_min

        # ── 读取融合状态 ──
        fused = robot.get_fused_data()
        theta_imu = fused.imu_yaw_unwrapped if fused.valid else 0.0
        omega     = fused.yaw_rate if fused.valid else 0.0

        # ── 可选：yaw 目标平滑（TrajectoryPlanner）──
        if planner is not None:
            yaw_pos_p, yaw_vel_p, yaw_acc_p, _ = planner.step(
                target_yaw, yaw_pos_p, yaw_vel_p, yaw_acc_p, DT_CTRL)
            mpc_target_yaw = yaw_pos_p
        else:
            mpc_target_yaw = target_yaw

        # ── 设置发送参数 + mpc 目标（后台线程 100Hz 求解并发送）──
        mcu_mpc.set(auto_aim_enable=1, yaw_torque_only_mode=0,
                    target_yaw=mpc_target_yaw, pitch_target_angle=target_pitch,
                    fire=0)

        # ── 最新 mpc 结果（后台线程更新）──
        mpc_state = mcu_mpc.get_state()

        # ── MCU 数据（pitch / temp）──
        data = robot.get_latest_data()
        cur_pitch = data.mcu_packet.pitch_angle if data.mcu_valid else 0.0

        # ── 波形历史（目标角度 = 延迟后的输入目标；目标速度 = mpc 解算值）──
        hist_delayed_target.push(mpc_state.delayed_target)
        hist_cur_yaw.push(theta_imu)
        hist_vel_sent.push(mpc_state.yaw_target_velocity)
        hist_omega.push(omega)
        hist_torque.push(mpc_state.yaw_torque)

        # ── 绘制 ──
        screen.fill(BLACK)
        mid = WINDOW_W // 2
        draw_angle_view(screen, mid // 2, CIRCLE_Y,
                        mpc_state.delayed_target, theta_imu,
                        "Yaw  (R=delayed target / B=current IMU)",
                        rotate_ccw90=True)
        draw_angle_view(screen, mid + mid // 2, CIRCLE_Y,
                        target_pitch, cur_pitch,
                        "Pitch  (R=target / B=current MCU)")

        if mode == 1:
            draw_control_box(screen, mx, my)

        draw_info(screen, fused, data, target_yaw, target_pitch, mpc_state,
                  int(1.0 / DT_CTRL) if total_time > 0.5 else 0, mode)

        wave_start_y = 620
        pygame.draw.line(screen, DGRAY, (0, wave_start_y), (WINDOW_W, wave_start_y), 2)

        wave_h = 135
        wave_gap = 12
        y0 = wave_start_y + 6
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Angle (delayed target + current)", hist_delayed_target, "rad",
                      RED, extra_traces=[(hist_cur_yaw, BLUE)])
        y0 += wave_h + wave_gap
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Angular Velocity (mpc target + actual)", hist_vel_sent, "rad/s",
                      GREEN, *VEL_Y_RANGE, extra_traces=[(hist_omega, ORANGE)])
        y0 += wave_h + wave_gap
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Torque (mpc)", hist_torque, "N\u00b7m",
                      YELLOW, *TORQUE_Y_RANGE)

        pygame.display.flip()

    # 清理
    if pygame.event.get_grab():
        pygame.event.set_grab(False)
    pygame.mouse.set_visible(True)
    robot.stop()
    robot.close()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
