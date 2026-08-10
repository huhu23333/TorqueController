"""
pygame_control.py — 使用 pygame 鼠标控制 yaw / pitch 的演示程序
                   yaw 由 StepRefinementWrapper 包装的 TrajectoryPlanner 求解

依赖: pip install pygame
运行: python3 -m scripts.pygame_control  (从 python/ 目录)
  或  PYTHONPATH=python python3 python/scripts/pygame_control.py

操作:
  点击窗口 → 锁定鼠标，移动鼠标控制目标角度
    - 水平位移 → yaw 目标 (左移 = +)
    - 竖向位移 → pitch 目标 (上移 = +), 限定 [-10°, +20°]
  ESC / 关闭窗口 → 退出

上方视图: 红色=目标 yaw  蓝色=MCU 当前 yaw  |  红色=目标 pitch 蓝色=MCU 当前 pitch
下方波形: 角度(发送+接收叠图) / 角速度(发送+接收叠图) / 力矩
"""

import sys, os, math, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from torque_controller import RobotCommunication, McuSendPacket, ImuSendPacket
from scripts.trajectory_planner import TrajectoryPlanner, StepRefinementWrapper

import pygame
from pygame.locals import *
from collections import deque

# ============================================================================
# 配置
# ============================================================================
WINDOW_W, WINDOW_H = 1920, 1080
FPS = 100
DT = 1.0 / FPS

LINE_LEN = 200          # 线段长度 (像素)
CIRCLE_R = 10            # 圆心半径
CIRCLE_Y = 200           # 角度视图圆心 Y 坐标
PITCH_MIN_DEG = -10.0   # pitch 下限 (度)
PITCH_MAX_DEG =  20.0   # pitch 上限 (度)
YAW_SENS   =  0.003     # 鼠标水平每像素 → yaw rad
PITCH_SENS =  0.003     # 鼠标竖向每像素 → pitch rad

# 轨迹规划器参数 (与 trajectory_viz 一致)
MAX_VEL   = 30.0       # rad/s
MAX_ACCEL = 50.0       # rad/s²
MAX_JERK  = 2000.0     # rad/s³
REFINE_N  = 1000        # StepRefinementWrapper 细化系数

# 波形图参数
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

# 绝对模式控制框
BOX_W, BOX_H = 900, 120
BOX_X = (WINDOW_W - BOX_W) // 2
BOX_Y = 460

# ============================================================================
# 滚动波形缓冲区
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
# 绘图
# ============================================================================
def draw_angle_view(surf, cx, cy, target_angle, current_angle, title, scale=1.0, rotate_ccw90=False):
    """以 (cx,cy) 为圆心画两条射线表示目标和当前角度"""
    t_ang = target_angle * scale
    c_ang = current_angle * scale
    # 目标 (红)
    if rotate_ccw90:
        ex = cx - LINE_LEN * math.sin(t_ang)
        ey = cy - LINE_LEN * math.cos(t_ang)
    else:
        ex = cx + LINE_LEN * math.cos(t_ang)
        ey = cy - LINE_LEN * math.sin(t_ang)
    pygame.draw.line(surf, RED, (cx, cy), (int(ex), int(ey)), 3)
    # 当前 (蓝)
    if rotate_ccw90:
        ex = cx - LINE_LEN * math.sin(c_ang)
        ey = cy - LINE_LEN * math.cos(c_ang)
    else:
        ex = cx + LINE_LEN * math.cos(c_ang)
        ey = cy - LINE_LEN * math.sin(c_ang)
    pygame.draw.line(surf, BLUE, (cx, cy), (int(ex), int(ey)), 3)
    # 圆心
    pygame.draw.circle(surf, WHITE, (cx, cy), CIRCLE_R)
    # 标题
    font = pygame.font.SysFont(None, 22)
    s = font.render(title, True, WHITE)
    surf.blit(s, (cx - s.get_width() // 2, cy + LINE_LEN + 10))

def draw_info(surf, data, target_yaw, target_pitch, yaw_torque, mode, fps, send_ok):
    font = pygame.font.SysFont(None, 20)
    y = 5
    def t(txt):
        nonlocal y
        s = font.render(txt, True, WHITE)
        surf.blit(s, (5, y)); y += 22

    imu = data.imu_packet; mcu = data.mcu_packet
    mode_names = {0: "RELATIVE (click lock)", 1: "ABSOLUTE (box)"}
    t(f"FPS: {fps:.0f}  |  Mode: {mode_names.get(mode, '?')}  |  [TAB:switch ESC:quit R:reset]")
    t(f"IMU Gyro: x={imu.gx:+.3f}  y={imu.gy:+.3f}  z={imu.gz:+.3f} rad/s")
    t(f"IMU Yaw: {imu.euler_yaw:7.3f} rad ({math.degrees(imu.euler_yaw):6.1f}°)  |  Pitch: {imu.euler_pitch:.3f}")
    t(f"Target  Yaw: {target_yaw:7.3f} rad ({math.degrees(target_yaw):6.1f}°)  |  Pitch: {target_pitch:.3f}")
    t(f"MCU Pitch: {mcu.pitch_angle:.3f}  |  Yaw Torque: {yaw_torque:+5.3f}")
    t(f"IMU valid: {data.imu_valid}  |  MCU valid: {data.mcu_valid}  |  Send OK: {send_ok}")
    t(f"MCU auto_aim_switch: {mcu.auto_aim_switch}  |  mark: {mcu.mark}  |  temp: {mcu.yaw_temperature}°C")


def draw_control_box(surf, mx, my):
    """绘制绝对模式控制框和十字准星"""
    r = pygame.Rect(BOX_X, BOX_Y, BOX_W, BOX_H)
    pygame.draw.rect(surf, GRAY, r, 1)
    # 中心十字线
    cx = BOX_X + BOX_W // 2; cy = BOX_Y + BOX_H // 2
    pygame.draw.line(surf, DGRAY, (BOX_X, cy), (BOX_X + BOX_W, cy), 1)
    pygame.draw.line(surf, DGRAY, (cx, BOX_Y), (cx, BOX_Y + BOX_H), 1)
    # 标签
    font = pygame.font.SysFont(None, 18)
    s = font.render("yaw: L=+pi  R=-pi", True, GRAY)
    surf.blit(s, (BOX_X, BOX_Y - 18))
    s = font.render("pitch: up=+  down=-", True, GRAY)
    surf.blit(s, (BOX_X + BOX_W - s.get_width(), BOX_Y + BOX_H + 2))
    # 十字准星（钳位到框内）
    cx_clamp = max(BOX_X, min(BOX_X + BOX_W, mx))
    cy_clamp = max(BOX_Y, min(BOX_Y + BOX_H, my))
    pygame.draw.line(surf, YELLOW, (cx_clamp - 10, cy_clamp), (cx_clamp + 10, cy_clamp), 2)
    pygame.draw.line(surf, YELLOW, (cx_clamp, cy_clamp - 10), (cx_clamp, cy_clamp + 10), 2)

def draw_waveform(surf, x, y, w, h, title, buf, y_label,
                  line_color, y_min=None, y_max=None,
                  extra_traces=None):
    """下方：滚动波形图，支持多条曲线叠加
       extra_traces: [(buf, color), ...] 额外叠加的曲线
    """
    rect = pygame.Rect(x, y, w, h)
    pygame.draw.rect(surf, DGRAY, rect, 1)
    pygame.draw.rect(surf, BLACK, rect.inflate(-2, -2))

    font = pygame.font.SysFont(None, 16)
    s = font.render(title, True, WHITE)
    surf.blit(s, (x + 4, y + 2))

    # 图例色块
    legend_x = x + s.get_width() + 10
    traces = [(buf, line_color)] + (extra_traces or [])
    for _, c in traces:
        pygame.draw.rect(surf, c, (legend_x, y + 5, 10, 10))
        legend_x += 14
    s = font.render(y_label, True, GRAY)
    surf.blit(s, (x + w - s.get_width() - 4, y + 2))

    # 合并 Y 轴范围
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

    # 水平网格线
    for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        gy = int(y + h - 4 - frac * (h - 28))
        pygame.draw.line(surf, LGRAY, (x + 2, gy), (x + w - 2, gy), 1)
        val = y_min + frac * (y_max - y_min)
        sv = font.render(f"{val:.1f}", True, GRAY)
        surf.blit(sv, (x + w - sv.get_width() - 4, gy - sv.get_height()))

    # 折线
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
# 主循环
# ============================================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Yaw / Pitch Mouse Control")
    clock = pygame.time.Clock()

    # 通信
    robot = RobotCommunication()
    print("Waiting for IMU & MCU data...")
    while True:
        data = robot.get_latest_data()
        if data.imu_valid and data.mcu_valid:
            break
        time.sleep(0.01)
    print("Data received. Starting control loop.")

    # 轨迹规划器 (参数与 trajectory_viz 一致)
    planner = TrajectoryPlanner(max_velocity=MAX_VEL, max_acceleration=MAX_ACCEL, max_jerk=MAX_JERK)
    refined_planner = StepRefinementWrapper(planner.step, REFINE_N)

    # Yaw 状态：位置使用接收到的 MCU yaw_angle 初始化
    data = robot.get_latest_data()
    yaw_pos = float(data.mcu_packet.yaw_angle)
    yaw_vel = 0.0
    yaw_acc = 0.0

    mode = 0                     # 0=相对(锁定)  1=绝对(框)
    target_yaw = yaw_pos         # 初始化目标 = 当前 MCU yaw
    target_pitch = 0.0
    pitch_min = math.radians(PITCH_MIN_DEG)
    pitch_max = math.radians(PITCH_MAX_DEG)

    # 波形历史缓冲区
    hist_sent_angle    = RingBuffer(HISTORY_N, yaw_pos)
    hist_mcu_angle     = RingBuffer(HISTORY_N, yaw_pos)
    hist_mcu_omega     = RingBuffer(HISTORY_N, 0.0)
    hist_sent_velocity = RingBuffer(HISTORY_N, 0.0)
    hist_torque        = RingBuffer(HISTORY_N, 0.0)

    # Y 轴范围
    VEL_Y_RANGE  = (-MAX_VEL * 1.2, MAX_VEL * 1.2)
    TORQUE_Y_RANGE = (-2.0, 2.0)

    running = True

    while running:
        dt = DT

        mx, my = pygame.mouse.get_pos()

        # ── 事件处理 ──
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
            elif evt.type == MOUSEBUTTONDOWN and mode == 0:
                pygame.mouse.set_visible(False)
                pygame.event.set_grab(True)

        # ── 目标角度 (去除 fmod 归一化) ──
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

        # ── 获取数据 ──
        data = robot.get_latest_data()

        # ── 轨迹规划器求解 ──
        yaw_pos, yaw_vel, yaw_acc, jerk = refined_planner.step(
            target_yaw, yaw_pos, yaw_vel, yaw_acc, dt
        )

        yaw_torque = 0.0

        # ── 发送 ──
        pkt = McuSendPacket(
            auto_aim_enable=1,
            pitch_target_angle=target_pitch,
            yaw_torque_only_mode=0,
            yaw_target_angle=yaw_pos,
            yaw_target_velocity=yaw_vel,
            yaw_torque=yaw_torque,
            fire=0,
        )
        ok = robot.send_to_mcu(pkt)

        # ── 更新波形历史 ──
        mcu_yaw = float(data.mcu_packet.yaw_angle) if data.mcu_valid else 0.0
        mcu_omega = float(data.mcu_packet.yaw_omega) if data.mcu_valid else 0.0
        hist_sent_angle.push(yaw_pos)
        hist_mcu_angle.push(mcu_yaw)
        hist_mcu_omega.push(mcu_omega)
        hist_sent_velocity.push(yaw_vel)
        hist_torque.push(yaw_torque)

        # ── 绘制 ──
        screen.fill(BLACK)
        mid = WINDOW_W // 2

        cur_pitch = float(data.mcu_packet.pitch_angle) if data.mcu_valid else 0.0
        draw_angle_view(screen, mid // 2, CIRCLE_Y,
                        target_yaw, mcu_yaw, "Yaw  (R=target / B=current MCU)",
                        rotate_ccw90=True)
        draw_angle_view(screen, mid + mid // 2, CIRCLE_Y,
                        target_pitch, cur_pitch, "Pitch  (R=target / B=current MCU)")

        if mode == 1:
            draw_control_box(screen, mx, my)

        draw_info(screen, data, target_yaw, target_pitch, yaw_torque, mode,
                  clock.get_fps(), ok)

        # 分隔线
        wave_start_y = 620
        pygame.draw.line(screen, DGRAY, (0, wave_start_y), (WINDOW_W, wave_start_y), 2)

        # 下方：3 个波形图
        wave_h = 135
        wave_gap = 12
        y0 = wave_start_y + 6
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Angle (sent+recv)", hist_sent_angle, "rad",
                      GREEN, y_min=None, y_max=None,
                      extra_traces=[(hist_mcu_angle, BLUE)])
        y0 += wave_h + wave_gap
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Angular Velocity (sent+recv)", hist_sent_velocity, "rad/s",
                      RED, *VEL_Y_RANGE,
                      extra_traces=[(hist_mcu_omega, ORANGE)])
        y0 += wave_h + wave_gap
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Torque (sent)", hist_torque, "N·m",
                      YELLOW, *TORQUE_Y_RANGE)

        pygame.display.flip()
        clock.tick(FPS)

    # 清理
    if pygame.event.get_grab():
        pygame.event.set_grab(False)
    pygame.mouse.set_visible(True)
    robot.stop()
    robot.close()
    pygame.quit()

if __name__ == "__main__":
    main()
