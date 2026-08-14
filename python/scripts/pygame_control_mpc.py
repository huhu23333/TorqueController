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
MAX_OMEGA       = 30.0   # 最高速度 (rad/s)
MAX_TORQUE      = 1.0    # 最大力矩 (N·m)
MAX_TORQUE_RATE = 10.0   # 最大力矩变化率 (N·m/s)

TWO_PI = 2.0 * math.pi


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def busy_wait_until(target_s):
    """忙等待到绝对时间点"""
    while time.perf_counter_ns() < target_s * 1e9:
        pass


# ================== 曲线绘图器类 (仿照 test3.py) ==================
class CurvePlotter:
    def __init__(self, screen, rect, dt, init_time_range=5.0, init_angle_range=3.14):
        self.screen = screen
        self.rect = rect
        self.dt = dt
        self.time_range = init_time_range
        self.angle_range = init_angle_range

        max_len = int(init_time_range / dt) + 100
        self.angles = deque(maxlen=max_len)
        self.targets = deque(maxlen=max_len)
        self.errors = deque(maxlen=max_len)
        self.font = pygame.font.SysFont("Consolas", 16)

    def add_point(self, angle_wrapped, target_angle):
        error = target_angle - angle_wrapped
        error = (error + math.pi) % (2 * math.pi) - math.pi
        self.angles.append(angle_wrapped)
        self.targets.append(target_angle)
        self.errors.append(error)

        needed_len = int(self.time_range / self.dt) + 10
        if self.angles.maxlen < needed_len:
            self.angles = deque(self.angles, maxlen=needed_len)
            self.targets = deque(self.targets, maxlen=needed_len)
            self.errors = deque(self.errors, maxlen=needed_len)

    def modify_time_range(self, delta):
        new_range = self.time_range + delta
        if 0.5 <= new_range <= 20.0:
            self.time_range = new_range
            new_len = int(self.time_range / self.dt) + 10
            self.angles = deque(self.angles, maxlen=new_len)
            self.targets = deque(self.targets, maxlen=new_len)
            self.errors = deque(self.errors, maxlen=new_len)

    def modify_angle_range(self, delta):
        new_range = self.angle_range + delta
        if 0.2 <= new_range <= math.pi:
            self.angle_range = new_range

    def draw(self):
        pygame.draw.rect(self.screen, (30, 30, 40), self.rect)
        pygame.draw.rect(self.screen, (100, 100, 120), self.rect, 2)

        plot_rect = self.rect.inflate(-40, -40)
        plot_rect.x += 20
        plot_rect.y += 20
        if plot_rect.width <= 0 or plot_rect.height <= 0:
            return

        n_h_lines = 5
        for i in range(n_h_lines + 1):
            y_ratio = i / n_h_lines
            y = plot_rect.bottom - y_ratio * plot_rect.height
            angle_val = -self.angle_range + 2 * self.angle_range * y_ratio
            pygame.draw.line(self.screen, (60, 60, 70),
                             (plot_rect.left, y), (plot_rect.right, y), 1)
            label = self.font.render(f"{angle_val:.1f}", True, (180, 180, 200))
            self.screen.blit(label, (plot_rect.left - 35, y - 5))

        n_v_lines = 6
        for i in range(n_v_lines + 1):
            x_ratio = i / n_v_lines
            x = plot_rect.left + x_ratio * plot_rect.width
            t_val = self.time_range * (1 - x_ratio)
            pygame.draw.line(self.screen, (60, 60, 70),
                             (x, plot_rect.top), (x, plot_rect.bottom), 1)
            if i % 2 == 0:
                label = self.font.render(f"{t_val:.1f}s", True, (180, 180, 200))
                self.screen.blit(label, (x - 20, plot_rect.bottom + 5))

        n_points = int(self.time_range / self.dt)
        angles_list = list(self.angles)[-n_points:]
        targets_list = list(self.targets)[-n_points:]
        errors_list = list(self.errors)[-n_points:]
        if len(angles_list) < 2:
            return

        def angle_to_y(angle):
            ratio = (angle + self.angle_range) / (2 * self.angle_range)
            ratio = max(0.0, min(1.0, ratio))
            return plot_rect.bottom - ratio * plot_rect.height

        def index_to_x(idx):
            ratio = idx / (len(angles_list) - 1)
            return plot_rect.left + ratio * plot_rect.width

        points_angle = [(index_to_x(i), angle_to_y(angles_list[i]))
                        for i in range(len(angles_list))]
        pygame.draw.lines(self.screen, (0, 200, 0), False, points_angle, 2)

        points_target = [(index_to_x(i), angle_to_y(targets_list[i]))
                         for i in range(len(targets_list))]
        pygame.draw.lines(self.screen, (200, 50, 50), False, points_target, 2)

        points_error = [(index_to_x(i), angle_to_y(errors_list[i]))
                        for i in range(len(errors_list))]
        pygame.draw.lines(self.screen, (240, 220, 60), False, points_error, 2)

        legend_y = self.rect.top + 10
        for color, text in [((0, 200, 0), "Actual"), ((200, 50, 50), "Target"), ((240, 220, 60), "Error")]:
            pygame.draw.rect(self.screen, color, (self.rect.right - 70, legend_y, 12, 12))
            label = self.font.render(text, True, (220, 220, 220))
            self.screen.blit(label, (self.rect.right - 55, legend_y - 2))
            legend_y += 18

        help_text = self.font.render(f"TimeRange:{self.time_range:.1f}s  AngleRange:{self.angle_range:.1f}rad", True, (200, 200, 200))
        self.screen.blit(help_text, (self.rect.x + 10, self.rect.bottom - 45))
        help2 = self.font.render("Keys: +/- : TimeRange  [ / ] : AngleRange  ESC : quit", True, (150, 150, 150))
        self.screen.blit(help2, (self.rect.x + 10, self.rect.bottom - 30))


# ================== 左侧绘图函数 (显示延迟后的目标) ==================
def draw_original(screen, font, angle, delayed_target, total_time, tau, omega, already_exceeded_time):
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
    screen.blit(time_surf, (10, 10))
    screen.blit(angle_surf, (10, 50))
    screen.blit(tau_surf, (10, 90))


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
        # 实控模式：连接 MCU 串口
        robot = RobotCommunication()
        print("Waiting for IMU & MCU data...")
        while True:
            data = robot.get_latest_data()
            if data.mcu_valid:
                break
            time.sleep(0.01)
        print("MCU data received. Starting MPC control loop.")
        # 当前状态（来自 MCU 编码器，多圈连续）
        theta = float(data.mcu_packet.yaw_angle)
        omega = float(data.mcu_packet.yaw_omega)

    # MPC 控制器（辨识参数，tau_d=0）
    mpc = MPCController(
        dt_control=DT_CTRL,
        dt_sim=DT_MPC_SIM,
        J=J, tau_c=TAU_C, b=B_FRIC, tau_d=TAU_D,
        max_omega=MAX_OMEGA, max_torque=MAX_TORQUE, max_torque_rate=MAX_TORQUE_RATE,
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

        # ----- 读取状态（仿真=环境推进, 实控=MCU 编码器）-----
        if sim_mode:
            # 应用上一步 MPC 输出的力矩，推进一个控制周期
            theta, omega = env.step(tau, DT_CTRL)
        else:
            data = robot.get_latest_data()
            if data.mcu_valid:
                theta = float(data.mcu_packet.yaw_angle)
                omega = float(data.mcu_packet.yaw_omega)

        # ----- 目标延迟缓冲 -----
        target_buffer.append((total_time, target_yaw))
        delayed_target = target_buffer[0][1]
        while len(target_buffer) > mpc.N:
            target_buffer.popleft()

        # ----- MPC 求解（参考轨迹：长度 N，恒定取延迟目标，多圈不归一化）-----
        ref = []
        for i in range(len(target_buffer)):
            ref.append(target_buffer[i][1])
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
        theta_wrapped = wrap_angle(theta)
        curve_plotter.add_point(theta_wrapped, wrap_angle(delayed_target))

        # ----- 渲染 -----
        screen.fill((20, 20, 20))
        draw_original(screen, font, theta_wrapped, wrap_angle(delayed_target), total_time, tau, omega, already_exceeded_time)
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
