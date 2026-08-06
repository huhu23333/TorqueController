// pitch_calibration.cpp
// Pitch 轴标定程序
// 采样 x(pitch_target_angle) 与 y(pitch_angle) 的关系，进行线性拟合
//
// 使用 0.1 和 0.9 分位值而非直接使用 min/max 的原因：
// 物理系统在行程端点附近通常存在非线性（如机械限位、电机力矩饱和、
// 传感器边缘效应等），端点处的测量值噪声也更大。取 0.1~0.9 分位范围
// 可以剔除两端各 10% 的不可靠数据，聚焦在线性度最好的中间区域进行
// 拟合，得到的斜率和截距更能代表系统的真实线性特性，避免端点异常值
// 拉偏回归结果。

#include "Communications.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <thread>
#include <csignal>

namespace {

void signalHandler(int) { 
    std::cout << "\n用户中断。\n";
    exit(0);
}

struct DataPoint { float x; float y; };

struct LinearFit {
    float slope     = 0;
    float intercept = 0;
    float r_squared = 0;
};

class PitchCalibrator {
public:
    PitchCalibrator(float x_min, float x_max, int fit_points = 20);
    ~PitchCalibrator();
    void run();

private:
    McuCommunication serial_;
    std::mutex              data_mutex_;
    mcu::ReceivePacket      latest_packet_{};
    bool                    has_data_ = false;

    float x_min_, x_max_;
    int   fit_points_;

    void onPacket(const mcu::ReceivePacket& pkt);
    float measureY(float x);
    float binarySearchX(float target_y, float x_lo, float x_hi,
                         float sign, int max_iter = 8);
    LinearFit fitLinear(const std::vector<DataPoint>& pts) const;
    static float lerp(float a, float b, float t) { return a + t * (b - a); }
};

PitchCalibrator::PitchCalibrator(float x_min, float x_max, int fit_points)
    : serial_([this](const mcu::ReceivePacket& pkt) { onPacket(pkt); })
    , x_min_(x_min), x_max_(x_max), fit_points_(fit_points)
{}

PitchCalibrator::~PitchCalibrator() { serial_.stopWorker(); }

void PitchCalibrator::onPacket(const mcu::ReceivePacket& pkt) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_packet_ = pkt;
    has_data_ = true;
}

float PitchCalibrator::measureY(float x) {
    mcu::SendPacket pkt;
    pkt.auto_aim_enable    = 1;
    pkt.pitch_target_angle = x;
    pkt.yaw_torque         = 0;
    pkt.fire               = 0;
    serial_.sendData(pkt);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    float y = 0;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (has_data_) y = latest_packet_.pitch_angle;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return y;
}

float PitchCalibrator::binarySearchX(float target_y,
                                      float x_lo, float x_hi,
                                      float sign, int max_iter) {
    float lo = x_lo, hi = x_hi;
    for (int i = 0; i < max_iter; ++i) {
        float mid = (lo + hi) * 0.5f;
        float y   = measureY(mid);
        std::cout << "    [" << i + 1 << "/" << max_iter << "] x="
                  << std::fixed << std::setprecision(4) << mid
                  << " -> y=" << y << " (target=" << target_y << ")\n";
        if (sign * (y - target_y) < 0) lo = mid;
        else                           hi = mid;
    }
    return (lo + hi) * 0.5f;
}

LinearFit PitchCalibrator::fitLinear(const std::vector<DataPoint>& pts) const {
    LinearFit result;
    size_t n = pts.size();
    if (n < 2) return result;
    float sx = 0, sy = 0, sxy = 0, sx2 = 0, sy2 = 0;
    for (auto& p : pts) {
        sx  += p.x;  sy  += p.y;
        sxy += p.x * p.y;
        sx2 += p.x * p.x;
        sy2 += p.y * p.y;
    }
    float denom = n * sx2 - sx * sx;
    if (std::fabs(denom) < 1e-9f) return result;
    result.slope     = (n * sxy - sx * sy) / denom;
    result.intercept = (sy - result.slope * sx) / n;
    float my = sy / n, ssr = 0, sst = 0;
    for (auto& p : pts) {
        float pred = result.slope * p.x + result.intercept;
        ssr += (p.y - pred) * (p.y - pred);
        sst += (p.y - my)   * (p.y - my);
    }
    result.r_squared = (sst > 1e-9f) ? 1.0f - ssr / sst : 1.0f;
    return result;
}

void PitchCalibrator::run() {
    serial_.startWorker();

    // ── Step 1: 测量两端点 ──
    std::cout << "\n========== Step 1: 测量两端点 ==========\n";
    std::cout << "x_min = " << x_min_ << ", x_max = " << x_max_ << "\n";

    float y_left  = measureY(x_min_);
    float y_right = measureY(x_max_);

    std::cout << "y_left  = " << y_left  << "  (at x=" << x_min_ << ")\n";
    std::cout << "y_right = " << y_right << "  (at x=" << x_max_ << ")\n";

    float sign = (y_right > y_left) ? 1.0f : -1.0f;
    std::cout << "相关性: " << (sign > 0 ? "正相关" : "负相关") << "\n";

    // ── Step 2: 二分查找 y 中心值对应的 x ──
    float y_mid = (y_left + y_right) * 0.5f;
    std::cout << "\n========== Step 2: 二分查找 y 中心 ==========\n";
    std::cout << "y_mid = " << y_mid << "\n";

    float x_center = binarySearchX(y_mid, x_min_, x_max_, sign, 8);
    std::cout << "x_center = " << x_center << "\n";

    // ── Step 3: 二分查找 0.1 / 0.9 分位值对应的 x ──
    float y_min = std::min(y_left, y_right);
    float y_max = std::max(y_left, y_right);
    float y_01 = lerp(y_min, y_max, 0.1f);
    float y_09 = lerp(y_min, y_max, 0.9f);

    std::cout << "\n========== Step 3: 查找 0.1 / 0.9 分位 ==========\n";
    std::cout << "y 范围: [" << y_min << ", " << y_max << "]\n";
    std::cout << "y_0.1 = " << y_01 << ", y_0.9 = " << y_09 << "\n";

    float x_01, x_09;
    if (y_left < y_right) {
        std::cout << "\n--- 向 x_min 方向搜索 x_0.1 ---\n";
        x_01 = binarySearchX(y_01, x_min_, x_center, sign, 8);
        std::cout << "--- 向 x_max 方向搜索 x_0.9 ---\n";
        x_09 = binarySearchX(y_09, x_center, x_max_, sign, 8);
    } else {
        std::cout << "\n--- 向 x_max 方向搜索 x_0.1 ---\n";
        x_01 = binarySearchX(y_01, x_center, x_max_, sign, 8);
        std::cout << "--- 向 x_min 方向搜索 x_0.9 ---\n";
        x_09 = binarySearchX(y_09, x_min_, x_center, sign, 8);
    }

    std::cout << "x_0.1 = " << x_01 << ", x_0.9 = " << x_09 << "\n";

    // ── Step 4: 在 [x_01, x_09] 范围内交替采样 ──
    float x_slo = std::min(x_01, x_09);
    float x_shi = std::max(x_01, x_09);

    std::cout << "\n========== Step 4: 交替采样拟合数据 ==========\n";
    std::cout << "采样范围: [" << x_slo << ", " << x_shi << "]\n";

    // 构建交替测量顺序：从两端向中间交替取值，避免单向漂移引入系统误差
    std::vector<float> x_order;
    x_order.reserve(fit_points_);
    {
        int lo = 0, hi = fit_points_ - 1;
        while (lo <= hi) {
            float t_lo = float(lo) / std::max(1, fit_points_ - 1);
            x_order.push_back(lerp(x_slo, x_shi, t_lo));
            lo++;
            if (lo > hi) break;
            float t_hi = float(hi) / std::max(1, fit_points_ - 1);
            x_order.push_back(lerp(x_slo, x_shi, t_hi));
            hi--;
        }
    }

    std::vector<DataPoint> samples;
    for (int i = 0; i < fit_points_; ++i) {
        float x = x_order[i];
        float y = measureY(x);
        samples.push_back({x, y});
        std::cout << "  [" << std::setw(2) << i + 1 << "/" << fit_points_ << "]"
                  << " x=" << std::fixed << std::setprecision(4) << x
                  << " -> y=" << y << "\n";
    }

    // ── Step 5: 线性拟合 ──
    std::cout << "\n========== Step 5: 线性拟合结果 ==========\n";
    LinearFit fit = fitLinear(samples);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "斜率 (slope)      : " << fit.slope << "\n";
    std::cout << "截距 (intercept)  : " << fit.intercept << "\n";
    std::cout << "R²                : " << fit.r_squared << "\n";

    // y 极值取自初始边界（已在 Step 3 计算），x 由拟合参数反推
    float x_at_y_min = (std::fabs(fit.slope) > 1e-9f)
                       ? (y_min - fit.intercept) / fit.slope : x_min_;
    float x_at_y_max = (std::fabs(fit.slope) > 1e-9f)
                       ? (y_max - fit.intercept) / fit.slope : x_max_;

    std::cout << std::setprecision(4);
    std::cout << "y 最小值          : " << y_min
              << "  (x=" << x_at_y_min << ")\n";
    std::cout << "y 最大值          : " << y_max
              << "  (x=" << x_at_y_max << ")\n";
    std::cout << "拟合公式: y = " << fit.slope << " * x + " << fit.intercept << "\n";

    std::cout << "\n========================================\n";
    std::cout << "标定完成。\n";
}

} // namespace

int main() {
    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    // ─── 硬编码参数（按需修改） ───
    constexpr float x_min =  -10.0f;
    constexpr float x_max =  30.0f;
    constexpr int   n_pts =  20;

    std::cout << "Pitch 轴标定程序\n";
    std::cout << "x 范围: [" << x_min << ", " << x_max << "]\n";
    std::cout << "拟合采样点数: " << n_pts << "\n";
    std::cout << "按 Ctrl+C 可随时中断\n";

    PitchCalibrator calib(x_min, x_max, n_pts);
    calib.run();

    return 0;
}
