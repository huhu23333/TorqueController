#include "mpc_controller.h"
#include <vector>

extern "C" {

// 不透明指针类型
typedef void* MpcHandle;

// step 返回结构：第一步控制力矩 + 预测位置 + 预测速度
typedef struct {
    double torque;
    double theta;
    double omega;
} MpcControl_C;

MpcHandle mpc_create(double dt_control, double dt_sim,
                     double J, double tau_c, double b, double tau_d,
                     double max_omega, double max_torque, double max_torque_rate,
                     int N, double Q, double R, double Rd, int max_iter) {
    try {
        MPCController* mpc = new MPCController(dt_control, dt_sim,
                                              J, tau_c, b, tau_d,
                                              max_omega, max_torque, max_torque_rate,
                                              N, Q, R, Rd, max_iter);
        return static_cast<MpcHandle>(mpc);
    } catch (...) {
        return nullptr;
    }
}

void mpc_destroy(MpcHandle handle) {
    if (handle) {
        delete static_cast<MPCController*>(handle);
    }
}

MpcControl_C mpc_step(MpcHandle handle, double theta, double omega,
                      const double* theta_ref, int ref_len) {
    MpcControl_C out;
    out.torque = 0.0;
    out.theta  = theta;
    out.omega  = omega;
    if (!handle) return out;

    MPCController* mpc = static_cast<MPCController*>(handle);
    std::vector<double> ref(theta_ref, theta_ref + ref_len);
    try {
        MPCController::Result r = mpc->step(theta, omega, ref);
        out.torque = r.torque;
        out.theta  = r.theta;
        out.omega  = r.omega;
    } catch (...) {
        // 求解异常时保持默认值（0 力矩 + 当前状态）
    }
    return out;
}

} // extern "C"
