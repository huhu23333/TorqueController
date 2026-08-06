// Com.cpp
#include "Com.h"

std::string SerialCommunicationClass::getSerialProductInfo(const std::string& port) {
    struct udev *udev;
    struct udev_device *dev;
    std::string result = "";
    
    udev = udev_new();
    if (!udev) {
        return "Failed to create udev";
    }
    
    dev = udev_device_new_from_subsystem_sysname(udev, "tty", port.c_str());
    if (!dev) {
        udev_unref(udev);
        return "Device not found";
    }
    
    struct udev_device *parent = udev_device_get_parent_with_subsystem_devtype(
        dev, "usb", "usb_device");
    
    if (parent) {
        const char *product = udev_device_get_sysattr_value(parent, "product");
        if (product) {
            result += std::string(product);
        }
    }
    
    udev_device_unref(dev);
    udev_unref(udev);
    return result;
}

SerialCommunicationClass::SerialCommunicationClass(std::function<void(const ReceivePacket&)> serialDataCallback) 
: serialDataCallback(serialDataCallback), fd_(-1) {
    initializeSerial();
    last_reconnect_time = std::chrono::steady_clock::now();
    last_received_time = std::chrono::steady_clock::now();
}

SerialCommunicationClass::~SerialCommunicationClass() {
    stop();
}

void SerialCommunicationClass::start() {
    running = true;
    recv_thread_ = std::thread(&SerialCommunicationClass::timerThread, this);
}

void SerialCommunicationClass::stop() {
    running = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

void SerialCommunicationClass::tryReconnect() {
    if (fd_ >= 0) {
        close(fd_);
    }
    buffer_index_ = 0;
    initializeSerial();
    last_reconnect_time = std::chrono::steady_clock::now();
    last_received_time = std::chrono::steady_clock::now();
}
    
void SerialCommunicationClass::initializeSerial() {
    std::vector<std::string> ports = findAvailableSerialPorts();
    if (ports.empty()) {
        printf("No available serial port found!\n");
        return;
    }
    std::string port;
    for (auto test_port : ports) {
        try {
            if(getSerialProductInfo(test_port.substr(5)) != std::string("STM32 Virtual ComPort MyIMU")) {
                port = test_port;
                break;
            };
        } catch (...) {

        }
    }
    if (port.empty()) {
        printf("Target serial port Not found!\n");
        return;
    }

    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
        printf("Failed to open port %s: %s\n", port.c_str(), strerror(errno));
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0) {
        printf("Failed to get serial attributes\n");
        close(fd_);
        fd_ = -1;
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        printf("Failed to set serial attributes\n");
        close(fd_);
        fd_ = -1;
        return;
    }

    tcflush(fd_, TCIOFLUSH);
}

std::vector<std::string> SerialCommunicationClass::findAvailableSerialPorts() {
    struct dirent *entry;
    DIR *dp = opendir("/dev/");
    if (dp == nullptr) {
        printf("Failed to open /dev/ directory\n");
        return std::vector<std::string>(0);
    }

    std::vector<std::string> ports;
    while ((entry = readdir(dp)) != nullptr) {
        if (strncmp(entry->d_name, "ttyACM", 6) == 0) {
            std::string candidate_port = "/dev/" + std::string(entry->d_name);
            int fd = open(candidate_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            if (fd >= 0) {
                close(fd);
                ports.push_back(candidate_port);
            }
        }
    }

    closedir(dp);
    return ports;
}

bool SerialCommunicationClass::sendData(SendPacket& packet) {
    if (fd_ >= 0) {
        packet.crc8 = CRC8_Check_Sum(reinterpret_cast<uint8_t*>(&packet), sizeof(SendPacket) - 1);

        ssize_t written = write(fd_, &packet, sizeof(SendPacket));
        if (written == static_cast<ssize_t>(sizeof(SendPacket))) {
            return true;
        } else {
            printf("TX write failed: written %ld bytes, expected %zu\n", 
                    written, sizeof(SendPacket));
        }
    }
    return false;
}

void SerialCommunicationClass::processFrame(const uint8_t* data) {
    uint8_t data_length = data[3];
    size_t frame_length = data_length + 5;
    size_t receive_packet_size = sizeof(ReceivePacket);
    
    if (frame_length == receive_packet_size) {
        ReceivePacket packet;
        memcpy(&packet, data, frame_length);
        serialDataCallback(packet);
    } else {
        ReceivePacket packet{};
        memcpy(&packet, data, frame_length < receive_packet_size ? frame_length : receive_packet_size);
        serialDataCallback(packet);
    }

    last_received_time = std::chrono::steady_clock::now();
}

void SerialCommunicationClass::processBuffer() {
    
    static const size_t MAX_FRAMES_PER_LOOP = 10;
    size_t frames_processed = 0;

    while (buffer_index_ >= FRAME_MIN_SIZE && frames_processed < MAX_FRAMES_PER_LOOP) {
        if (buffer_index_ >= BUFFER_SIZE - 128) {
            printf("Buffer approaching capacity (%zu bytes), clearing\n", buffer_index_);
            buffer_index_ = 0;
            return;
        }

        size_t header_pos = 0;
        bool found_header = false;
        
        while (header_pos <= buffer_index_ - 3 && header_pos < 128) {
            if (buffer_[header_pos] == FRAME_HEADER1 && 
                buffer_[header_pos + 1] == FRAME_HEADER2 && 
                buffer_[header_pos + 2] == PROTOCOL_VERSION) {
                found_header = true;
                break;
            }
            header_pos++;
        }

        if (!found_header) {
            if (buffer_index_ > 2) {
                buffer_[0] = buffer_[buffer_index_ - 2];
                buffer_[1] = buffer_[buffer_index_ - 1];
                buffer_index_ = 2;
            }
            return;
        }

        if (header_pos > 0) {
            if (header_pos < buffer_index_) {
                memmove(buffer_.data(), buffer_.data() + header_pos, buffer_index_ - header_pos);
                buffer_index_ -= header_pos;
            } else {
                buffer_index_ = 0;
                return;
            }
        }

        if (buffer_index_ < 4) {
            return;
        }

        uint8_t data_length = buffer_[3];
        size_t frame_length = data_length + 5;

        if (data_length > MAX_FRAME_LENGTH || frame_length > BUFFER_SIZE) {
            printf("Invalid frame length detected: %zu (data_length=%u, max=%zu)\n",
                    frame_length, data_length, MAX_FRAME_LENGTH);
            buffer_index_ = 0;
            return;
        }

        if (buffer_index_ < frame_length) {
            return;
        }

        if (CRC8_Check_Sum(buffer_.data(), frame_length - 1) == buffer_[frame_length - 1]) {
            processFrame(buffer_.data());
            frames_processed++;
        } else {
            printf("CRC check failed, discarding frame\n");
            memmove(buffer_.data(), buffer_.data() + 3, buffer_index_ - 3);
            buffer_index_ -= 3;
            continue;
        }

        if (frame_length < buffer_index_) {
            memmove(buffer_.data(), buffer_.data() + frame_length, buffer_index_ - frame_length);
            buffer_index_ -= frame_length;
        } else {
            buffer_index_ = 0;
        }
    }
}

void SerialCommunicationClass::timerCallback() {
    if (fd_ < 0) {
        if (std::chrono::steady_clock::now() - last_reconnect_time > std::chrono::seconds(3)) {
            printf("Serial port not available, trying reconnect\n");
            tryReconnect();
        }
        return;
    }
    if (std::chrono::steady_clock::now() - last_received_time > std::chrono::seconds(3)) {
        if (std::chrono::steady_clock::now() - last_reconnect_time > std::chrono::seconds(3)) {
            printf("No data received, trying reconnect\n");
            tryReconnect();
        }
        return;
    }

    if (buffer_index_ < BUFFER_SIZE - 128) {
        uint8_t temp_buffer[128];
        ssize_t bytes_read = read(fd_, temp_buffer, sizeof(temp_buffer));
        
        if (bytes_read > 0) {
            if (buffer_index_ + bytes_read < BUFFER_SIZE) {
                memcpy(buffer_.data() + buffer_index_, temp_buffer, bytes_read);
                buffer_index_ += bytes_read;
                processBuffer();
            } else {
                printf("Buffer near full, discarding data\n");
                buffer_index_ = 0;
            }
        }
    }
}

void SerialCommunicationClass::timerThread() {
    while (running) {
        auto start = std::chrono::steady_clock::now();

        timerCallback();

        std::this_thread::sleep_until(start + std::chrono::microseconds(100));
    }
}
