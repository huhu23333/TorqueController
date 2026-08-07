"""
param_ident.py — 系统辨识（基于 data/sysid_samples/*.npz）

模型: J * dω/dt = τ - τ_c * sign(ω) - b * ω + τ_d
使用 PyTorch 可微分仿真 + tanh 软符号函数进行梯度优化
"""

import torch, torch.nn as nn
import numpy as np, os, sys, math, matplotlib.pyplot as plt

SAVE_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "data", "sysid_samples")
DT = 0.01

# ============================================================================
# 可微分仿真环境
# ============================================================================
class DifferentiableYawSimEnv:
    def __init__(self, dt=DT):
        self.dt = dt

    def set_params(self, J, tau_c, b, tau_d):
        self.J, self.tau_c, self.b, self.tau_d = J, tau_c, b, tau_d

    def simulate(self, tau_seq, omega_init):
        N = tau_seq.shape[0]
        omega = omega_init.clone().detach()
        omega_list = []
        lam = 1e4
        for i in range(N):
            soft_sign = torch.tanh(lam * omega)
            tau_f = -soft_sign * self.tau_c - self.b * omega
            tau_net = tau_seq[i] + tau_f + self.tau_d
            alpha = tau_net / self.J
            omega = omega + alpha * self.dt
            omega_list.append(omega)
        return torch.stack(omega_list)

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
            'yaw': d['yaw'], 'gz': d['gz'],
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
        opt.zero_grad()
        total_loss = torch.tensor(0.0, device=device)
        n_seq = 0

        for sample in samples:
            torque = sample['torque']
            L = len(torque)
            if L < 2:
                continue
            gz = sample['gz']

            tau_t = torch.tensor(torque, dtype=torch.float32, device=device)
            omega_init = torch.tensor(float(gz[0]), dtype=torch.float32, device=device)
            omega_true = torch.tensor(gz, dtype=torch.float32, device=device)

            J, tc, b, td = get_params()
            diff_env.set_params(J, tc, b, td)
            omega_sim = diff_env.simulate(tau_t, omega_init)

            loss = torch.mean((omega_sim - omega_true) ** 2)
            total_loss = total_loss + loss
            n_seq += 1

        if n_seq == 0:
            continue
        total_loss = total_loss / n_seq
        total_loss.backward()
        opt.step()

        loss_history.append(total_loss.item())
        J, tc, b, td = get_params()
        param_history.append([J.item(), tc.item(), b.item(), td.item()])

        if epoch % 500 == 0:
            print(f"Epoch {epoch:5d}  loss={total_loss.item():.6f}  "
                  f"J={J.item():.4f} tau_c={tc.item():.3f} b={b.item():.4f} tau_d={td.item():.4f}")

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
    final_params, loss_history, param_history = optimize(samples, num_epochs=5000, lr=3e-4, device=device)

    param_names = ['J', 'tau_c', 'b', 'tau_d']
    print("\n" + "=" * 50)
    print("Identified parameters:")
    for name, val in zip(param_names, final_params):
        print(f"  {name} = {val:.6f}")
    print("=" * 50)

    # 画图
    param_history = np.array(param_history)
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))

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
    gz_true = torch.tensor(sample['gz'], dtype=torch.float32, device=device)
    omega_init = torch.tensor(float(sample['gz'][0]), dtype=torch.float32, device=device)
    with torch.no_grad():
        gz_sim = diff_env.simulate(tau_t, omega_init)

    axes[1, 2].plot(gz_true.cpu()[:300], label='Real', alpha=0.7)
    axes[1, 2].plot(gz_sim.cpu()[:300], '--', label='Sim', alpha=0.7)
    axes[1, 2].set_title('gz compare'); axes[1, 2].legend(); axes[1, 2].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
