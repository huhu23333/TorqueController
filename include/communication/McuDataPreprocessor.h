#pragma once

#include "Protocol.hpp"
#include <cmath>

// ============================================================================
// McuDataPreprocessor — MCU 通信数据预处理类
// ============================================================================
class McuDataPreprocessor {
public:

    // ── 发送包预处理 ──
    static mcu::SendPacket processSend(const mcu::SendPacket& packet);

    // ── 接收包预处理 ──
    static mcu::ReceivePacket processReceive(const mcu::ReceivePacket& packet);
};
