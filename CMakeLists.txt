cmake_minimum_required(VERSION 3.10)
project(TorqueController LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找udev库
find_library(UDEV_LIBRARY udev)
if(NOT UDEV_LIBRARY)
    message(FATAL_ERROR "libudev not found")
endif()

# 通信库源文件
set(COMMUNICATION_SOURCES
    src/CRC.cpp
    src/Com.cpp
)

# 通信库头文件
set(COMMUNICATION_HEADERS
    include/CRC.h
    include/Com.h
    Protocol/v1.hpp
)

# 构建通信库
add_library(communication STATIC ${COMMUNICATION_SOURCES})
target_include_directories(communication PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(communication PUBLIC ${UDEV_LIBRARY})

# 构建测试可执行文件
add_executable(test_serial main.cpp)
target_link_libraries(test_serial PRIVATE communication)

# 构建 Pitch 轴标定程序
add_executable(pitch_calibration pitch_calibration.cpp)
target_link_libraries(pitch_calibration PRIVATE communication)
