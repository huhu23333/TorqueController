cmake_minimum_required(VERSION 3.10)
project(TorqueController LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# 查找udev库
find_library(UDEV_LIBRARY udev)
if(NOT UDEV_LIBRARY)
    message(FATAL_ERROR "libudev not found")
endif()

# 通信库源文件（CRC 为核心，模板类为 header-only）
set(COMMUNICATION_SOURCES
    src/CRC.cpp
    src/McuDataPreprocessor.cpp
    src/FusionFilter.cpp
)

# 通信库头文件
set(COMMUNICATION_HEADERS
    include/CRC.h
    include/SerialProtocol.hpp
    include/Protocol.hpp
    include/Communications.hpp
    include/McuDataPreprocessor.h
    include/FusionFilter.h
)

# 构建通信库
add_library(communication STATIC ${COMMUNICATION_SOURCES})
target_include_directories(communication PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(communication PUBLIC ${UDEV_LIBRARY})

# 构建测试可执行文件
add_executable(test_serial src/test_serial.cpp)
target_link_libraries(test_serial PRIVATE communication)

# 构建 Pitch 轴标定程序
add_executable(pitch_calibration src/pitch_calibration.cpp)
target_link_libraries(pitch_calibration PRIVATE communication)

# 构建 C API 动态库（供 Python ctypes 加载；含实车 MPC 控制闭环）
find_package(Ceres REQUIRED)

add_library(robot_comm_c SHARED
    src/RobotCommunicationC.cpp
    ${COMMUNICATION_SOURCES}
    mpc/mpc_controller.cpp
    mpc/yaw_mpc_controller.cpp
    mpc/mcu_mpc_controller.cpp
)
target_include_directories(robot_comm_c PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/mpc
    ${CERES_INCLUDE_DIRS}
)
target_link_libraries(robot_comm_c PUBLIC ${UDEV_LIBRARY} ${CERES_LIBRARIES})

# 构建控制演示程序
add_executable(control_demo src/control_demo.cpp)
target_link_libraries(control_demo PRIVATE communication)
