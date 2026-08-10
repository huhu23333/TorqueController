"""
trajectory_viz.py — 轨迹规划器 pygame 可视化

- 鼠标横向坐标映射到 0~4π (两整圈，不归一化到 0~2π) 作为目标位置
- 100 Hz 运行，将目标送入 TrajectoryPlanner，维护位置/速度状态
- 左侧：两条共端点射线表示目标位置(红)和实际位置(蓝)的朝向
- 右侧：目标位置、当前位置、速度、加速度的滚动波形图

依赖: pip install pygame
运行: python3 python/scripts/trajectory_viz.py
"""

import sys, os, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scripts.trajectory_planner import TrajectoryPlanner, StepRefinementWrapper
import pygame
from pygame.locals import *
from collections import deque

# ============================================================================
# 配置
# ============================================================================
WINDOW_W, WINDOW_H = 1100, 600
FPS = 100
DT = 1.0 / FPS

# 轨迹规划器参数
MAX_VEL   = 12.0       # rad/s
MAX_ACCEL = 20.0       # rad/s²
MAX_JERK  = 80.0       # rad/s³

# 目标范围：鼠标横向 → 0 ~ 4π (两整圈)
TARGET_MIN = 0.0
TARGET_MAX = 4.0 * math.pi

# 角度视图参数
CIRCLE_CX = 210
CIRCLE_CY = 270
LINE_LEN  = 160
CIRCLE_R  = 8

# 波形图参数
WAVE_LEFT   = 450
WAVE_RIGHT  = 1080
WAVE_WIDTH  = WAVE_RIGHT - WAVE_LEFT
HISTORY_S   = 3.0                     # 显示 3 秒历史
HISTORY_N   = int(HISTORY_S * FPS)    # 300 个点

# 颜色
RED      = (255, 60, 60)
BLUE     = (60, 120, 255)
GREEN    = (100, 220, 100)
ORANGE   = (255, 160, 40)
YELLOW   = (220, 220, 60)
WHITE    = (240, 240, 240)
BLACK    = (18, 18, 18)
DGRAY    = (45, 45, 45)
GRAY     = (100, 100, 100)
LGRAY    = (65, 65, 65)

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
# 绘图辅助
# ============================================================================
def angle_to_xy(cx, cy, r, angle_rad):
    """角度 → 屏幕坐标（0 rad = 右侧 = 3点钟方向，逆时针）"""
    x = cx + r * math.cos(angle_rad)
    y = cy - r * math.sin(angle_rad)
    return int(x), int(y)


def draw_angle_view(surf, target_angle, current_angle):
    """左侧：两条共端点射线 + 参考圆"""
    cx, cy = CIRCLE_CX, CIRCLE_CY
    r = LINE_LEN

    # 参考圆
    pygame.draw.circle(surf, DGRAY, (cx, cy), r, 2)
    # 十字参考线
    pygame.draw.line(surf, DGRAY, (cx - r, cy), (cx + r, cy), 1)
    pygame.draw.line(surf, DGRAY, (cx, cy - r), (cx, cy + r), 1)

    # 目标射线 (红色)
    tx, ty = angle_to_xy(cx, cy, r, target_angle)
    pygame.draw.line(surf, RED, (cx, cy), (tx, ty), 3)

    # 实际射线 (蓝色)
    cx2, cy2 = angle_to_xy(cx, cy, r, current_angle)
    pygame.draw.line(surf, BLUE, (cx, cy), (cx2, cy2), 3)

    # 圆心
    pygame.draw.circle(surf, WHITE, (cx, cy), CIRCLE_R)

    # 标签
    font = pygame.font.SysFont(None, 20)
    s = font.render("RED=target  BLUE=actual", True, WHITE)
    surf.blit(s, (cx - s.get_width() // 2, cy + r + 8))

    # 4 个主方向刻度标注 (0, π, 2π, 3π, 4π)
    font_s = pygame.font.SysFont(None, 16)
    for label, ang in [("0", 0), ("\u03c0", math.pi), ("2\u03c0", 2*math.pi),
                        ("3\u03c0", 3*math.pi), ("4\u03c0", 4*math.pi)]:
        norm = ang % (2 * math.pi)
        tx2, ty2 = angle_to_xy(cx, cy, r + 16, norm)
        s2 = font_s.render(label, True, GRAY)
        surf.blit(s2, (tx2 - s2.get_width() // 2, ty2 - s2.get_height() // 2))


def draw_waveform(surf, x, y, w, h, title, buf, y_label,
                  line_color, y_min=None, y_max=None):
    """右侧：单个滚动波形图"""
    rect = pygame.Rect(x, y, w, h)
    pygame.draw.rect(surf, DGRAY, rect, 1)
    pygame.draw.rect(surf, BLACK, rect.inflate(-2, -2))

    font = pygame.font.SysFont(None, 18)
    s = font.render(title, True, WHITE)
    surf.blit(s, (x + 4, y + 2))
    s = font.render(y_label, True, GRAY)
    surf.blit(s, (x + w - s.get_width() - 4, y + 2))

    data = buf.data()
    n = len(data)
    if n < 2:
        return

    if y_min is None or y_max is None:
        y_min, y_max = buf.min_max()
    else:
        y_min, y_max = float(y_min), float(y_max)

    # 水平网格线
    for frac in [0.0, 0.25, 0.5, 0.75, 1.0]:
        gy = int(y + h - 4 - frac * (h - 28))
        pygame.draw.line(surf, LGRAY, (x + 2, gy), (x + w - 2, gy), 1)
        val = y_min + frac * (y_max - y_min)
        sv = font.render(f"{val:.1f}", True, GRAY)
        surf.blit(sv, (x + w - sv.get_width() - 4, gy - sv.get_height()))

    # 折线
    points = []
    for i, val in enumerate(data):
        frac_x = i / max(n - 1, 1)
        frac_y = (val - y_min) / max(y_max - y_min, 1e-9)
        px = int(x + 6 + frac_x * (w - 14))
        py = int(y + h - 8 - frac_y * (h - 32))
        points.append((px, py))

    if len(points) >= 2:
        pygame.draw.lines(surf, line_color, False, points, 2)


def draw_info_bar(surf, target, pos, vel, acc, jerk, fps):
    """底部信息栏"""
    font = pygame.font.SysFont(None, 20)
    y = WINDOW_H - 26
    texts = [
        f"FPS: {fps:.0f}",
        f"Target: {target:.3f} rad = {target/(2*math.pi):.2f} rev",
        f"Pos:    {pos:.3f} rad = {pos/(2*math.pi):.2f} rev",
        f"Vel:    {vel:+.3f} rad/s",
        f"Acc:    {acc:+.3f} rad/s\u00b2",
        f"Jerk:   {jerk:+.1f} rad/s\u00b3",
    ]
    total_w = sum(font.render(t, True, WHITE).get_width() + 16 for t in texts)
    start_x = (WINDOW_W - total_w) // 2
    for t in texts:
        s = font.render(t, True, WHITE)
        surf.blit(s, (start_x, y))
        start_x += s.get_width() + 16

    hint = "[ESC:quit]  [R:reset to 0]  \u2190 mouse X \u2192 target 0~4\u03c0"
    font_s = pygame.font.SysFont(None, 16)
    hs = font_s.render(hint, True, GRAY)
    surf.blit(hs, ((WINDOW_W - hs.get_width()) // 2, WINDOW_H - 8))

# ============================================================================
# 主循环
# ============================================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Trajectory Planner Visualizer")
    clock = pygame.time.Clock()

    planner = TrajectoryPlanner(max_velocity=MAX_VEL, max_acceleration=MAX_ACCEL, max_jerk=MAX_JERK)
    refined_planner = StepRefinementWrapper(planner.step, 10)

    # 状态
    target_pos = 2.0 * math.pi       # 默认 1 圈
    current_pos = 2.0 * math.pi
    current_vel = 0.0
    current_acc = 0.0

    # 历史缓冲区
    hist_target = RingBuffer(HISTORY_N, target_pos)
    hist_pos    = RingBuffer(HISTORY_N, current_pos)
    hist_vel    = RingBuffer(HISTORY_N, 0.0)
    hist_acc    = RingBuffer(HISTORY_N, 0.0)
    hist_jerk   = RingBuffer(HISTORY_N, 0.0)

    # 固定 Y 轴范围
    POS_Y_RANGE  = (TARGET_MIN - 0.5, TARGET_MAX + 0.5)
    VEL_Y_RANGE  = (-MAX_VEL * 1.2, MAX_VEL * 1.2)
    ACC_Y_RANGE  = (-MAX_ACCEL * 1.2, MAX_ACCEL * 1.2)
    JERK_Y_RANGE = (-MAX_JERK * 1.2, MAX_JERK * 1.2)

    running = True
    while running:
        dt_ms = clock.tick(FPS)
        fps = clock.get_fps()

        # ── 事件处理 ──
        for evt in pygame.event.get():
            if evt.type == QUIT:
                running = False
            elif evt.type == KEYDOWN:
                if evt.key == K_ESCAPE:
                    running = False
                elif evt.key == K_r:
                    target_pos = 0.0
                    current_pos = 0.0
                    current_vel = 0.0
                    current_acc = 0.0

        # ── 鼠标 → 目标位置 ──
        mx, _ = pygame.mouse.get_pos()
        mx_clamped = max(0, min(WINDOW_W, mx))
        target_pos = TARGET_MIN + (mx_clamped / WINDOW_W) * (TARGET_MAX - TARGET_MIN)

        # ── 轨迹规划器迭代 ──
        current_pos, current_vel, current_acc, jerk = refined_planner.step(
            target_pos, current_pos, current_vel, current_acc, DT
        )

        # ── 更新历史 ──
        hist_target.push(target_pos)
        hist_pos.push(current_pos)
        hist_vel.push(current_vel)
        hist_acc.push(current_acc)
        hist_jerk.push(jerk)

        # ── 绘制 ──
        screen.fill(BLACK)

        # 左侧：角度视图
        draw_angle_view(screen, target_pos, current_pos)

        # 中间分隔线
        mid_x = WAVE_LEFT - 10
        pygame.draw.line(screen, DGRAY, (mid_x, 0), (mid_x, WINDOW_H), 2)

        # 右侧：5 个波形图
        wave_h = 102
        wave_gap = 6
        y0 = 10
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Target Position", hist_target, "rad",
                      GREEN, *POS_Y_RANGE)
        y0 += wave_h + wave_gap
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Current Position", hist_pos, "rad",
                      BLUE, *POS_Y_RANGE)
        y0 += wave_h + wave_gap
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Velocity", hist_vel, "rad/s",
                      ORANGE, *VEL_Y_RANGE)
        y0 += wave_h + wave_gap
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Acceleration", hist_acc, "rad/s\u00b2",
                      RED, *ACC_Y_RANGE)
        y0 += wave_h + wave_gap
        draw_waveform(screen, WAVE_LEFT, y0, WAVE_WIDTH, wave_h,
                      "Jerk", hist_jerk, "rad/s\u00b3",
                      YELLOW, *JERK_Y_RANGE)

        # 底部信息栏
        draw_info_bar(screen, target_pos, current_pos,
                      current_vel, current_acc, jerk, fps)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()

