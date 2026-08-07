"""
pygame_control.py — 使用 pygame 鼠标控制 yaw / pitch 的演示程序

依赖: pip install pygame
运行: python3 -m scripts.pygame_control  (从 python/ 目录)
  或  PYTHONPATH=python python3 python/scripts/pygame_control.py

操作:
  点击窗口 → 锁定鼠标，移动鼠标控制目标角度
    - 水平位移 → yaw 目标 (左移 = +)
    - 竖向位移 → pitch 目标 (上移 = +), 限定 [-10°, +20°]
  ESC / 关闭窗口 → 退出

左侧视图: 红色=目标 yaw  蓝色=IMU 当前 yaw
右侧视图: 红色=目标 pitch 蓝色=MCU 当前 pitch
"""

import sys, os, math, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from torque_controller import RobotCommunication, McuSendPacket, ImuSendPacket

import pygame
from pygame.locals import *

# ============================================================================
# 配置
# ============================================================================
WINDOW_W, WINDOW_H = 900, 500
LINE_LEN = 150          # 线段长度 (像素)
CIRCLE_R = 8            # 圆心半径
PITCH_MIN_DEG = -10.0   # pitch 下限 (度)
PITCH_MAX_DEG =  20.0   # pitch 上限 (度)
YAW_SENS   =  0.003     # 鼠标水平每像素 → yaw rad
PITCH_SENS =  0.003    # 鼠标竖向每像素 → pitch rad

PID_KP, PID_KI, PID_KD = 2.0, 0.1, 0.2
PID_OUT_MIN, PID_OUT_MAX = -1.0, 1.0

RED    = (255, 60, 60)
BLUE   = (60, 120, 255)
WHITE  = (240, 240, 240)
BLACK  = (20, 20, 20)
GRAY   = (80, 80, 80)
DGRAY  = (40, 40, 40)
GREEN  = (100, 200, 100)

YELLOW = (220, 220, 60)

TWO_PI = 2.0 * math.pi

# 绝对模式控制框
BOX_W, BOX_H = 400, 120
BOX_X = (WINDOW_W - BOX_W) // 2
BOX_Y = WINDOW_H - BOX_H - 20

# ============================================================================
# PID
# ============================================================================
class PidController:
    def __init__(self, kp, ki, kd, out_min, out_max):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        deriv = (error - self.prev_error) / dt if dt > 1e-6 else 0.0
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * deriv
        sat_hi = out > self.out_max
        sat_lo = out < self.out_min
        if sat_hi: out = self.out_max
        if sat_lo: out = self.out_min
        do_int = True
        if sat_hi and error > 0: do_int = False
        if sat_lo and error < 0: do_int = False
        if do_int: self.integral += error * dt
        return out

    def reset(self):
        self.integral = 0.0; self.prev_error = 0.0



# ============================================================================
# 绘图
# ============================================================================
def draw_angle_view(surf, cx, cy, target_angle, current_angle, title, scale=1.0):
    """以 (cx,cy) 为圆心画两条射线表示目标和当前角度"""
    t_ang = target_angle * scale
    c_ang = current_angle * scale
    # 目标 (红)
    ex = cx + LINE_LEN * math.cos(t_ang)
    ey = cy - LINE_LEN * math.sin(t_ang)
    pygame.draw.line(surf, RED, (cx, cy), (int(ex), int(ey)), 3)
    # 当前 (蓝)
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
    t(f"IMU Yaw: {imu.euler_yaw:7.3f} rad ({math.degrees(imu.euler_yaw):6.1f}°)  |  Pitch: {imu.euler_pitch:.3f}")
    t(f"Target  Yaw: {target_yaw:7.3f} rad ({math.degrees(target_yaw):6.1f}°)  |  Pitch: {target_pitch:.3f}")
    t(f"MCU Pitch: {mcu.pitch_angle:.3f}  |  Yaw Torque: {yaw_torque:+5.3f}")
    t(f"IMU valid: {data.imu_valid}  |  MCU valid: {data.mcu_valid}  |  Send OK: {send_ok}")
    t(f"MCU auto_aim_switch: {mcu.auto_aim_switch}  |  mark: {mcu.mark}  |  color: {mcu.color}")


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

    pid = PidController(PID_KP, PID_KI, PID_KD, PID_OUT_MIN, PID_OUT_MAX)
    mode = 0                     # 0=相对(锁定)  1=绝对(框)
    target_yaw = 0.0
    target_pitch = 0.0
    pitch_min = math.radians(PITCH_MIN_DEG)
    pitch_max = math.radians(PITCH_MAX_DEG)

    last_time = time.monotonic()
    running = True

    while running:
        now = time.monotonic()
        dt = now - last_time
        last_time = now

        mx, my = pygame.mouse.get_pos()

        # ── 事件处理 ──
        for evt in pygame.event.get():
            if evt.type == QUIT:
                running = False
            elif evt.type == KEYDOWN:
                if evt.key == K_ESCAPE:
                    running = False
                elif evt.key == K_TAB:
                    # 切换模式
                    if mode == 0:
                        mode = 1
                        pygame.mouse.set_visible(True)
                        pygame.event.set_grab(False)
                        pid.reset()
                    else:
                        mode = 0
                elif evt.key == K_r:
                    pid.reset()
            elif evt.type == MOUSEBUTTONDOWN and mode == 0:
                pygame.mouse.set_visible(False)
                pygame.event.set_grab(True)

        # ── 目标角度 ──
        if mode == 0:
            # 相对模式：鼠标位移
            if pygame.event.get_grab():
                dx, dy = pygame.mouse.get_rel()
                target_yaw   -= float(dx) * YAW_SENS
                target_pitch -= float(dy) * PITCH_SENS
        else:
            # 绝对模式：鼠标在框内的位置
            fx = (mx - BOX_X) / BOX_W          # 0..1, 左=0 右=1
            fy = (my - BOX_Y) / BOX_H          # 0..1, 上=0 下=1
            fx = max(0.0, min(1.0, fx))
            fy = max(0.0, min(1.0, fy))
            target_yaw   = (1.0 - fx) * TWO_PI - math.pi   # 左=+pi, 右=-pi
            target_pitch = (1.0 - fy) * (pitch_max - pitch_min) + pitch_min  # 上=+, 下=-

        # yaw 归一化 / pitch 限位
        target_yaw = math.fmod(target_yaw + math.pi, TWO_PI)
        if target_yaw < 0: target_yaw += TWO_PI
        target_yaw -= math.pi
        if target_pitch > pitch_max: target_pitch = pitch_max
        if target_pitch < pitch_min: target_pitch = pitch_min

        # ── 获取数据 ──
        data = robot.get_latest_data()

        # ── PID ──
        yaw_torque = 0.0
        obs_yaw = 0.0; err = 0.0
        if data.imu_valid:
            obs_yaw = float(data.imu_packet.euler_yaw)
            err = math.remainder(target_yaw - obs_yaw, TWO_PI)
            yaw_torque = pid.update(err, dt)

        # ── 发送 ──
        pkt = McuSendPacket(auto_aim_enable=1, pitch_target_angle=target_pitch,
                            yaw_torque=yaw_torque, fire=0)
        ok = robot.send_to_mcu(pkt)

        # ── 绘制 ──
        screen.fill(BLACK)
        mid = WINDOW_W // 2
        pygame.draw.line(screen, DGRAY, (mid, 0), (mid, WINDOW_H), 2)

        cur_yaw   = float(data.imu_packet.euler_yaw) if data.imu_valid else 0.0
        cur_pitch = float(data.mcu_packet.pitch_angle) if data.mcu_valid else 0.0

        draw_angle_view(screen, mid // 2, WINDOW_H // 2,
                        target_yaw, cur_yaw, "Yaw  (R=target / B=current IMU)")
        draw_angle_view(screen, mid + mid // 2, WINDOW_H // 2,
                        target_pitch, cur_pitch, "Pitch  (R=target / B=current MCU)")

        if mode == 1:
            draw_control_box(screen, mx, my)

        draw_info(screen, data, target_yaw, target_pitch, yaw_torque, mode,
                  clock.get_fps(), ok)

        pygame.display.flip()
        clock.tick(100)

    # 清理
    if pygame.event.get_grab():
        pygame.event.set_grab(False)
    pygame.mouse.set_visible(True)
    robot.stop()
    robot.close()
    pygame.quit()

if __name__ == "__main__":
    main()
