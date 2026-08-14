"""
param_ident.py — 系统辨识（基于 data/sysid_samples/*.npz）

模型: J * dω/dt = τ - τ_c * sign(ω) - b * ω + τ_d
使用 PyTorch 可微分仿真 + tanh 软符号函数进行梯度优化
"""

import random
import torch, torch.nn as nn
import numpy as np, os, sys, math, matplotlib.pyplot as plt

seed = 42
random.seed(seed)
np.random.seed(seed)

SAVE_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "data", "sysid_samples")
DT = 0.01
SEG_STEPS = 10  # 每次拟合只截取 0.1s（10 步）的片段

# ============================================================================
# 可微分仿真环境
# ============================================================================
class DifferentiableYawSimEnv:
    def __init__(self, dt=DT):
        self.dt = dt

    def set_params(self, J, tau_c, b, tau_d):
        self.J, self.tau_c, self.b, self.tau_d = J, tau_c, b, tau_d

    def simulate(self, tau_seq, theta_init, omega_init):
        N = tau_seq.shape[0]
        theta = theta_init.clone().detach()
        omega = omega_init.clone().detach()
        theta_list = []
        omega_list = []
        lam = 1e4
        for i in range(N):
            soft_sign = torch.tanh(lam * omega)
            tau_f = -soft_sign * self.tau_c - self.b * omega
            tau_net = tau_seq[i] + tau_f + self.tau_d
            alpha = tau_net / self.J
            omega = omega + alpha * self.dt
            theta = theta + omega * self.dt
            theta_list.append(theta)
            omega_list.append(omega)
        return torch.stack(theta_list), torch.stack(omega_list)

# ============================================================================
# 数据加载
# ============================================================================
def load_samples():
    files = sorted(f for f in os.listdir(SAVE_DIR) if f.endswith('.npz'))
    samples = []
    for f in files:
        d = np.load(os.path.join(SAVE_DIR, f))
        samples.append({
            'target': d['target'], 'torque': d['torque'],
            'yaw': np.unwrap(d['yaw']), 'gz': d['gz'],
        })
    print(f"Loaded {len(samples)} samples")
    return samples

# ============================================================================
# 优化
# ============================================================================
def optimize(samples, num_epochs=5000, lr=1e-3, device='cpu'):
    diff_env = DifferentiableYawSimEnv(dt=DT)

    log_J = nn.Parameter(torch.tensor(math.log(0.05), device=device))
    log_tau_c = nn.Parameter(torch.tensor(math.log(0.5), device=device))
    log_b = nn.Parameter(torch.tensor(math.log(0.03), device=device))
    tau_d = nn.Parameter(torch.tensor(0.0, device=device))

    def get_params():
        return torch.exp(log_J), torch.exp(log_tau_c), torch.exp(log_b), tau_d

    opt = torch.optim.Adam([log_J, log_tau_c, log_b, tau_d], lr=lr)

    loss_history, param_history = [], []
    for epoch in range(num_epochs):
        epoch_loss = 0.0
        n_seq = 0

        for sample_idx, (sample) in enumerate(samples):
            torque = sample['torque']
            L = len(torque)
            if L < SEG_STEPS:
                continue
            gz = sample['gz']
            yaw = sample['yaw']

            # 每次随机截取 0.1s（10 步）片段，而不是使用整个片段
            start = random.randint(0, L - SEG_STEPS)
            torque_seg = torque[start:start + SEG_STEPS]
            gz_seg = gz[start:start + SEG_STEPS]
            yaw_seg = yaw[start:start + SEG_STEPS]

            tau_t = torch.tensor(torque_seg, dtype=torch.float32, device=device)
            theta_init = torch.tensor(float(yaw_seg[0]), dtype=torch.float32, device=device)
            omega_init = torch.tensor(float(gz_seg[0]), dtype=torch.float32, device=device)
            theta_true = torch.tensor(yaw_seg, dtype=torch.float32, device=device)
            omega_true = torch.tensor(gz_seg, dtype=torch.float32, device=device)

            J, tc, b, td = get_params()
            diff_env.set_params(J, tc, b, td)
            theta_sim, omega_sim = diff_env.simulate(tau_t, theta_init, omega_init)

            # 每模拟一个片段就计算一个片段的损失（位置 + 速度）
            # 角度误差先归一化到 (-π, π] 再平方，避免 ±π 处绕卷跳变
            delta = theta_sim - theta_true
            delta = torch.atan2(torch.sin(delta), torch.cos(delta))
            pos_loss = torch.mean(delta ** 2)
            vel_loss = torch.mean((omega_sim - omega_true) ** 2)
            loss = pos_loss + vel_loss

            opt.zero_grad()
            loss.backward()
            opt.step()

            # 每次 step 打印当前参数与该片段的 loss
            J_now, tc_now, b_now, td_now = get_params()

            epoch_loss += loss.item()
            n_seq += 1

        if n_seq == 0:
            continue
        epoch_loss /= n_seq

        print(f"Epoch {epoch:5d} loss={epoch_loss:.6f}  "
              f"J={J_now.item():.4f} tau_c={tc_now.item():.3f} "
              f"b={b_now.item():.4f} tau_d={td_now.item():.4f}")

        loss_history.append(epoch_loss)
        J, tc, b, td = get_params()
        param_history.append([J.item(), tc.item(), b.item(), td.item()])

    return [p.item() for p in get_params()], loss_history, param_history

# ============================================================================
# 主流程
# ============================================================================
def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"Device: {device}")

    samples = load_samples()
    if not samples:
        print("No samples found! Run collect_sysid_data.py first.")
        return

    print("Optimizing...")
    final_params, loss_history, param_history = optimize(samples, num_epochs=1000, lr=3e-4, device=device)

    param_names = ['J', 'tau_c', 'b', 'tau_d']
    print("\n" + "=" * 50)
    print("Identified parameters:")
    for name, val in zip(param_names, final_params):
        print(f"  {name} = {val:.6f}")
    print("=" * 50)

    # 画图
    param_history = np.array(param_history)
    fig, axes = plt.subplots(3, 3, figsize=(16, 10))

    axes[0, 0].plot(loss_history)
    axes[0, 0].set_yscale('log')
    axes[0, 0].set_title('Loss'); axes[0, 0].grid(True)

    for i, name in enumerate(param_names):
        ax = axes[(i+1)//3, (i+1)%3]
        ax.plot(param_history[:, i])
        ax.set_title(name); ax.grid(True)

    # 仿真对比（最后一条样本）
    diff_env = DifferentiableYawSimEnv(dt=DT)
    J, tc, b, td = [torch.tensor(v, device=device) for v in final_params]
    diff_env.set_params(J, tc, b, td)

    sample = samples[-1]
    tau_t = torch.tensor(sample['torque'], dtype=torch.float32, device=device)
    yaw_true = torch.tensor(sample['yaw'], dtype=torch.float32, device=device)
    gz_true = torch.tensor(sample['gz'], dtype=torch.float32, device=device)
    theta_init = torch.tensor(float(sample['yaw'][0]), dtype=torch.float32, device=device)
    omega_init = torch.tensor(float(sample['gz'][0]), dtype=torch.float32, device=device)
    with torch.no_grad():
        yaw_sim, gz_sim = diff_env.simulate(tau_t, theta_init, omega_init)

    # 位置（yaw）曲线对比
    axes[1, 2].plot(yaw_true.cpu()[:300], label='Real', alpha=0.7)
    axes[1, 2].plot(yaw_sim.cpu()[:300], '--', label='Sim', alpha=0.7)
    axes[1, 2].set_title('yaw compare'); axes[1, 2].legend(); axes[1, 2].grid(True)

    # 速度（gz）曲线对比
    axes[2, 0].plot(gz_true.cpu()[:300], label='Real', alpha=0.7)
    axes[2, 0].plot(gz_sim.cpu()[:300], '--', label='Sim', alpha=0.7)
    axes[2, 0].set_title('gz compare'); axes[2, 0].legend(); axes[2, 0].grid(True)

    # 删除多余的空白子图
    fig.delaxes(axes[2, 1])
    fig.delaxes(axes[2, 2])

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
