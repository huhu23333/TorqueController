// Com.h
#ifndef COM_H
#define COM_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <queue>
#include <array>
#include <mutex>
#include <atomic>
#include <chrono>
#include "CRC.h"
#include <dirent.h>  // 用于遍历/dev目录
#include <sys/types.h>
#include <sys/stat.h>
#include <functional>
#include <iostream>
#include <libudev.h>
#include <thread>
#include "Protocol/v1.hpp"

class SerialCommunicationClass {
public:
    SerialCommunicationClass(std::function<void(const ReceivePacket&)> serialDataCallback);
    ~SerialCommunicationClass();
    void start();
    void stop();
    void timerCallback();
    bool sendData(SendPacket& packet);

private:
    static constexpr size_t BUFFER_SIZE = 1024;
    static constexpr size_t MAX_FRAME_LENGTH = 64;
    static constexpr uint8_t FRAME_HEADER1 = 0x42;
    static constexpr uint8_t FRAME_HEADER2 = 0x52;
    static constexpr uint8_t PROTOCOL_VERSION = 0x01;
    static constexpr size_t FRAME_MIN_SIZE = 5;

    int fd_;
    std::array<uint8_t, BUFFER_SIZE> buffer_;
    size_t buffer_index_ = 0;

    std::function<void(const ReceivePacket&)> serialDataCallback;
    std::atomic<bool> running{true};
    std::thread recv_thread_;

    std::chrono::steady_clock::time_point last_reconnect_time;
    std::chrono::steady_clock::time_point last_received_time;
    
    void initializeSerial();
    std::vector<std::string> findAvailableSerialPorts();
    void processFrame(const uint8_t* data);
    void processBuffer();
    void tryReconnect();
    void timerThread();
    std::string getSerialProductInfo(const std::string& port);
};

#endif // COM_H
