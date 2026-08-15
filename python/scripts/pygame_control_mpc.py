"""
pygame_control_mpc.py — 使用 MPC 控制 yaw 的 pygame 演示程序

- 目标输入/可视化仿照 test3.py（鼠标目标 + 0.1s 目标延迟 + 箭头与曲线）
- 观测: MCU 的 yaw_angle（位置，rad，多圈连续）和 yaw_omega（速度，rad/s）
- 控制: MPC 求解第一步控制力矩 + 预测位置 + 预测速度，
        通过 yaw_torque_only_mode=0 一并发送给 MCU
- 模型参数取自 params/1/Identified_parameters.txt（tau_d=0）
- 时间控制: time.perf_counter_ns() + 忙等待，每帧对齐到 start + frame*DT_S
            （与 collect_sysid_data.py 一致），不再依赖 pygame 的 clock.tick
- 参考轨迹: 传入 mpc.step 的长度为 N（ref[i] 对应预测 theta_pred[i+1]），
            恒定取延迟目标；位置误差直接相减不归一化
- 角度语义: yaw_angle 与目标均为多圈连续角度（非 wrap 到 [-π,π]），
            MPC 内部全程按多圈处理

运行:
    实控模式:
        PYTHONPATH=python python3 python/scripts/pygame_control_mpc.py
    仿真模式 (--sim，使用与 MPC 相同参数的内部动力学环境，不连接硬件):
        PYTHONPATH=python python3 python/scripts/pygame_control_mpc.py --sim
"""

import sys, os, math, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from torque_controller import RobotCommunication, McuSendPacket
from torque_controller.mpc_wrapper import MPCController
from torque_controller.sim_yaw_env import SimYawEnv

import pygame
from pygame.locals import *
from collections import deque

# ================== 窗口与区域参数 (与 test3.py 相同) ==================
ORIGIN_WIDTH, ORIGIN_HEIGHT = 1600, 1200
CURVE_WIDTH = 800
WIDTH = ORIGIN_WIDTH + CURVE_WIDTH
HEIGHT = ORIGIN_HEIGHT
CENTER_X = ORIGIN_WIDTH // 2
CENTER_Y = HEIGHT // 2
B = 300
ARROW_LEN = 500
CURVE_RECT = pygame.Rect(ORIGIN_WIDTH, 0, CURVE_WIDTH, HEIGHT)

# ================== 控制参数 ==================
DT_CTRL = 0.01           # MPC 控制周期 (100 Hz，与 MCU 通信周期一致)
DT_S = DT_CTRL
DT_MPC_SIM = 0.01       # MPC 仿真步长（控制周期的）
DELAY_TIME = 0.2         # 目标延迟时间 (秒)
YAW_SENS = 0.003         # 鼠标每像素 -> yaw rad
MPC_PRED_N = int(DELAY_TIME / DT_CTRL)

# ================== 辨识参数 (params/1/Identified_parameters.txt) ==================
J       = 0.016541
TAU_C   = 0.097297
B_FRIC  = 0.032100
TAU_D   = 0.0            # tau_d 设为 0

# ================== 约束 ==================
MAX_TORQUE      = 1.0    # 最大力矩 (N·m)
MAX_TORQUE_RATE = 40.0   # 最大力矩变化率 (N·m/s)

TWO_PI = 2.0 * math.pi


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def busy_wait_until(target_s):
    """忙等待到绝对时间点"""
    while time.perf_counter_ns() < target_s * 1e9:
        pass


# ================== 曲线绘图器类（三面板波形：Angle / Velocity / Torque，仿 trajectory_viz.py）==================
class CurvePlotter:
    COLORS = {
        "actual": (0, 200, 0),        # 绿色
        "target": (200, 50, 50),      # 红色
        "error":  (240, 220, 60),     # 黄色
        "ctrl":   (60, 160, 255),     # 蓝色（控制量）
        "omega":  (255, 160, 40),     # 橙色（实际速度）
        "torque": (255, 120, 200),    # 品红（力矩）
    }

    def __init__(self, screen, rect, dt, init_time_range=5.0, init_angle_range=3.14):
        self.screen = screen
        self.rect = rect
        self.dt = dt
        self.time_range = init_time_range
        self.angle_range = init_angle_range

        max_len = int(init_time_range / dt) + 100
        self.angles     = deque(maxlen=max_len)
        self.targets    = deque(maxlen=max_len)
        self.errors     = deque(maxlen=max_len)
        self.omega_ctrl = deque(maxlen=max_len)   # 控制速度（MPC 预测第一步，发送值）
        self.omega_act  = deque(maxlen=max_len)   # 实际速度（MCU 编码器 / 仿真）
        self.torque     = deque(maxlen=max_len)   # 控制力矩（MPC 输出）
        self.font = pygame.font.SysFont("Consolas", 16)

    def add_point(self, angle_wrapped, target_angle, omega_ctrl, omega_act, torque):
        error = target_angle - angle_wrapped
        error = (error + math.pi) % (2 * math.pi) - math.pi
        self.angles.append(angle_wrapped)
        self.targets.append(target_angle)
        self.errors.append(error)
        self.omega_ctrl.append(omega_ctrl)
        self.omega_act.append(omega_act)
        self.torque.append(torque)

        needed_len = int(self.time_range / self.dt) + 10
        if self.angles.maxlen < needed_len:
            self._resize_buffers(needed_len)

    def _resize_buffers(self, new_len):
        for name in ("angles", "targets", "errors", "omega_ctrl", "omega_act", "torque"):
            setattr(self, name, deque(getattr(self, name), maxlen=new_len))

    def modify_time_range(self, delta):
        new_range = self.time_range + delta
        if 0.5 <= new_range <= 20.0:
            self.time_range = new_range
            self._resize_buffers(int(self.time_range / self.dt) + 10)

    def modify_angle_range(self, delta):
        new_range = self.angle_range + delta
        if 0.2 <= new_range <= math.pi:
            self.angle_range = new_range

    def draw(self):
        # 垂直三面板布局
        margin, gap, help_h = 4, 3, 24
        avail = self.rect.height - help_h
        panel_h = (avail - margin * 2 - gap * 2) // 3
        panels = []
        y = self.rect.top + margin
        for _ in range(3):
            panels.append(pygame.Rect(self.rect.left + margin, y,
                                      self.rect.width - margin * 2, panel_h))
            y += panel_h + gap

        # 角度面板（Y 范围可调，[ / ] 键）
        self._draw_panel(panels[0], "Angle", [
            (self.COLORS["actual"], "Actual", self.angles),
            (self.COLORS["target"], "Target", self.targets),
            (self.COLORS["error"],  "Error",  self.errors),
        ], "rad", fixed_range=(-self.angle_range, self.angle_range))

        # 速度面板（Y 范围自动缩放）：Ctrl = MPC 预测第一步速度，Actual = 实测速度
        self._draw_panel(panels[1], "Velocity", [
            (self.COLORS["ctrl"],  "Ctrl",   self.omega_ctrl),
            (self.COLORS["omega"], "Actual", self.omega_act),
        ], "rad/s", fixed_range=None)

        # 力矩面板（Y 范围固定 ±1.2，MAX_TORQUE=1.0）
        self._draw_panel(panels[2], "Torque", [
            (self.COLORS["torque"], "Ctrl", self.torque),
        ], "N\u00b7m", fixed_range=(-1.2, 1.2))

        # 底部帮助
        help_text = self.font.render(
            f"TimeRange:{self.time_range:.1f}s  AngleRange:{self.angle_range:.1f}rad   "
            f"Keys: +/- :Time  [ / ] :Angle  ESC : quit", True, (150, 150, 150))
        self.screen.blit(help_text, (self.rect.left + 8, self.rect.bottom - help_h + 5))

    def _draw_panel(self, rect, title, series, y_label, fixed_range=None):
        """绘制单个波形面板：标题/单位/图例 + 网格 + 各序列折线"""
        pygame.draw.rect(self.screen, (30, 30, 40), rect)
        pygame.draw.rect(self.screen, (100, 100, 120), rect, 2)

        # 标题（左上）与单位
        t = self.font.render(title, True, (220, 220, 220))
        self.screen.blit(t, (rect.left + 6, rect.top + 4))
        u = self.font.render(y_label, True, (150, 150, 150))
        self.screen.blit(u, (rect.left + 6 + t.get_width() + 8, rect.top + 4))

        # 图例（右上）
        ly = rect.top + 4
        for color, name, _ in series:
            pygame.draw.rect(self.screen, color, (rect.right - 76, ly + 4, 10, 10))
            lab = self.font.render(name, True, (220, 220, 220))
            self.screen.blit(lab, (rect.right - 62, ly))
            ly += 16

        plot_rect = rect.inflate(-28, -36)
        plot_rect.y += 18
        if plot_rect.width <= 8 or plot_rect.height <= 8:
            return

        # Y 范围：固定或自动（所有序列 min/max + 10% 边距）
        if fixed_range is not None:
            y_min, y_max = fixed_range
        else:
            vals = [v for _, _, dq in series for v in dq]
            if not vals:
                return
            y_min, y_max = min(vals), max(vals)
            if y_max - y_min < 1e-9:
                y_min -= 1.0
                y_max += 1.0
            mg = (y_max - y_min) * 0.1
            y_min -= mg
            y_max += mg

        # 水平网格 + Y 刻度
        for i in range(6):
            yr = i / 5
            gy = plot_rect.bottom - yr * plot_rect.height
            pygame.draw.line(self.screen, (60, 60, 70),
                             (plot_rect.left, gy), (plot_rect.right, gy), 1)
            val = y_min + yr * (y_max - y_min)
            lab = self.font.render(f"{val:.1f}", True, (180, 180, 200))
            self.screen.blit(lab, (plot_rect.left - lab.get_width() - 4, gy - 7))

        # 竖直时间网格 + 时间刻度
        for i in range(7):
            xr = i / 6
            gx = plot_rect.left + xr * plot_rect.width
            pygame.draw.line(self.screen, (60, 60, 70),
                             (gx, plot_rect.top), (gx, plot_rect.bottom), 1)
            if i % 2 == 0:
                tv = self.time_range * (1 - xr)
                lab = self.font.render(f"{tv:.1f}s", True, (180, 180, 200))
                self.screen.blit(lab, (gx - 12, plot_rect.bottom + 2))

        # 序列折线
        n_points = int(self.time_range / self.dt)
        for color, _name, dq in series:
            data = list(dq)[-n_points:]
            if len(data) < 2:
                continue
            pts = []
            for idx, val in enumerate(data):
                fx = idx / (len(data) - 1)
                fy = (val - y_min) / max(y_max - y_min, 1e-9)
                fy = max(0.0, min(1.0, fy))
                pts.append((int(plot_rect.left + fx * plot_rect.width),
                            int(plot_rect.bottom - fy * plot_rect.height)))
            pygame.draw.lines(self.screen, color, False, pts, 2)


# ================== 左侧绘图函数 (显示延迟后的目标) ==================
def draw_original(screen, font, angle, delayed_target, total_time, tau, omega, already_exceeded_time, temperature):
    line_y = CENTER_Y - B
    pygame.draw.line(screen, (180, 180, 180), (0, line_y), (ORIGIN_WIDTH, line_y), 1)

    # 当前角度箭头 (绿色)
    end_x = CENTER_X - ARROW_LEN * math.sin(angle)
    end_y = CENTER_Y - ARROW_LEN * math.cos(angle)
    pygame.draw.line(screen, (0, 255, 0), (CENTER_X, CENTER_Y), (end_x, end_y), 3)
    head_len = 12
    head_angle = math.pi / 7
    ang1 = angle + math.pi - head_angle
    ang2 = angle + math.pi + head_angle
    p1 = (end_x - head_len * math.sin(ang1), end_y - head_len * math.cos(ang1))
    p2 = (end_x - head_len * math.sin(ang2), end_y - head_len * math.cos(ang2))
    pygame.draw.polygon(screen, (0, 255, 0), [p1, (end_x, end_y), p2])

    # 目标方向指示点 (红色) - 使用延迟目标
    target_x = CENTER_X - ARROW_LEN * math.sin(delayed_target)
    target_y = CENTER_Y - ARROW_LEN * math.cos(delayed_target)
    pygame.draw.circle(screen, (255, 60, 60), (int(target_x), int(target_y)), 6)

    time_surf = font.render(f"Time: {total_time:.4f} s, Exceeded: {already_exceeded_time} s", True, (255, 255, 255))
    angle_surf = font.render(f"Angle: {angle:.3f}  Target: {delayed_target:.3f}", True, (255, 255, 255))
    tau_surf = font.render(f"Torque: {tau:+.3f}  Omega: {omega:+.3f} rad/s", True, (255, 255, 255))
    extra_surf = font.render(f"temperature: {temperature}°C", True, (255, 255, 255))
    screen.blit(time_surf, (10, 10))
    screen.blit(angle_surf, (10, 50))
    screen.blit(tau_surf, (10, 90))
    screen.blit(extra_surf, (10, 130))


# ================== 主函数 ==================
def main():
    sim_mode = "--sim" in sys.argv

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Yaw Control with MPC" + (" [SIM]" if sim_mode else " (real MCU)"))
    font = pygame.font.Font(None, 30)

    if sim_mode:
        # 仿真模式：使用与 MPC 相同参数的内部动力学环境，不连接硬件
        print("SIMULATION MODE — 使用内部动力学模型 (J={}, tau_c={}, b={})".format(J, TAU_C, B_FRIC))
        env = SimYawEnv(J, TAU_C, B_FRIC, TAU_D, dt_sim=DT_MPC_SIM)
        theta, omega = env.state
        robot = None
    else:
        # 实控模式：连接 MCU 串口，使用融合滤波器输出（高频 yaw + 底盘姿态）
        robot = RobotCommunication()
        print("Waiting for fused data (IMU + MCU yaw)...")
        while True:
            fused = robot.get_fused_data()
            if fused.valid:
                break
            time.sleep(0.01)
        print("Fused data ready. Starting MPC control loop.")
        # 当前状态（融合解卷绕位置 + 高频速度，多圈连续）
        theta = fused.yaw_pos
        omega = fused.yaw_rate

    # MPC 控制器（辨识参数，tau_d=0）
    mpc = MPCController(
        dt_control=DT_CTRL,
        dt_sim=DT_MPC_SIM,
        J=J, tau_c=TAU_C, b=B_FRIC, tau_d=TAU_D,
        max_torque=MAX_TORQUE, max_torque_rate=MAX_TORQUE_RATE,
        N=MPC_PRED_N, Q=5.0, R=0.01, Rd=0.1, max_iter=30,
    )

    curve_plotter = CurvePlotter(screen, CURVE_RECT, DT_CTRL,
                                 init_time_range=5.0, init_angle_range=3.14)

    # 目标（相对当前 yaw，连续多圈）
    target_yaw = theta
    # 目标延迟队列: (时间戳, 目标)
    target_buffer = deque()
    delayed_target = theta

    total_time = 0.0
    tau = 0.0
    running = True
    grabbed = False

    # ── 忙等待时间基准：每帧对齐到 start + frame*DT_S（与 collect_sysid_data.py 一致）──
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
        # 用理论流逝时间作为 total_time

        # ----- 事件处理 -----
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_EQUALS or event.key == pygame.K_PLUS:
                    curve_plotter.modify_time_range(0.5)
                elif event.key == pygame.K_MINUS:
                    curve_plotter.modify_time_range(-0.5)
                elif event.key == pygame.K_LEFTBRACKET:
                    curve_plotter.modify_angle_range(-0.2)
                elif event.key == pygame.K_RIGHTBRACKET:
                    curve_plotter.modify_angle_range(0.2)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                # 鼠标在左侧区域按下 -> 锁定鼠标进行相对控制
                if 0 <= event.pos[0] < ORIGIN_WIDTH:
                    grabbed = True
                    pygame.mouse.set_visible(False)
                    pygame.event.set_grab(True)
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                grabbed = False
                pygame.mouse.set_visible(True)
                pygame.event.set_grab(False)

        # ----- 鼠标相对运动更新目标 -----
        if grabbed:
            dx, _ = pygame.mouse.get_rel()
            target_yaw -= dx * YAW_SENS

        # ----- 读取状态（仿真=环境推进, 实控=融合滤波器输出）-----
        if sim_mode:
            # 应用上一步 MPC 输出的力矩，推进一个控制周期
            theta, omega = env.step(tau, DT_CTRL)
            temperature = 0
        else:
            fused = robot.get_fused_data()
            if fused.valid:
                theta = fused.yaw_pos          # 解卷绕多圈位置
                omega = fused.yaw_rate         # 高频速度
                theta_imu = fused.imu_yaw_unwrapped
                theta_chassis = fused.chassis_yaw
            data = robot.get_latest_data()     # 仅用于温度显示
            temperature = data.mcu_packet.yaw_temperature
            if data.mcu_valid:
                chassis_imu_omega = data.mcu_packet.chassis_imu_omega

        # ----- 目标延迟缓冲 -----
        target_buffer.append((total_time, target_yaw))
        delayed_target = target_buffer[0][1]
        while len(target_buffer) > mpc.N:
            target_buffer.popleft()

        # ----- MPC 求解（参考轨迹：长度 N，恒定取延迟目标，多圈不归一化）-----
        ref = []
        for i in range(len(target_buffer)):
            ref.append(target_buffer[i][1] - ((theta_imu - theta) + (i+1) * DT_CTRL * chassis_imu_omega))
        if len(ref) < MPC_PRED_N:
            ref = ref + [ref[-1]] * (MPC_PRED_N - len(ref))

        tau, theta_pred, omega_pred = mpc.step(theta, omega, ref)

        # ----- 发送给 MCU：力矩 + 位置 + 速度（yaw_torque_only_mode=0）-----
        if not sim_mode:
            pkt = McuSendPacket(
                auto_aim_enable=1,
                pitch_target_angle=0.0,
                yaw_torque_only_mode=0,
                yaw_target_angle=theta_pred,
                yaw_target_velocity=omega_pred,
                yaw_torque=tau,
                fire=0,
            )
            robot.send_to_mcu(pkt)

        # ----- 曲线数据（wrap 到 [-π, π] 用于显示）-----
        theta_wrapped = wrap_angle(theta_imu)
        curve_plotter.add_point(theta_wrapped, wrap_angle(delayed_target),
                                omega_pred, omega, tau)

        # ----- 渲染 -----
        screen.fill((20, 20, 20))
        draw_original(screen, font, theta_imu, delayed_target, total_time, tau, omega, already_exceeded_time, temperature)
        curve_plotter.draw()
        pygame.display.flip()

    # 清理
    pygame.mouse.set_visible(True)
    pygame.event.set_grab(False)
    if robot is not None:
        robot.stop()
        robot.close()
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
