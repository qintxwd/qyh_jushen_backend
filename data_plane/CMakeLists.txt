cmake_minimum_required(VERSION 3.16)
project(qyh_data_plane VERSION 2.0.0 LANGUAGES CXX)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# Release 模式优化
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ==================== 依赖查找 ====================

# Boost
find_package(Boost REQUIRED COMPONENTS system thread)

# Protobuf
find_package(Protobuf REQUIRED)

# Threads
find_package(Threads REQUIRED)

# yaml-cpp (配置文件解析)
find_package(yaml-cpp REQUIRED)

# OpenSSL (JWT)
find_package(OpenSSL REQUIRED)

# JSON (Control Sync)
find_package(nlohmann_json 3.9 REQUIRED)

# ROS2 (可选)
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    message(STATUS "Building with ROS2 support")
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(tf2_msgs REQUIRED)
    find_package(trajectory_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    
    # Custom Messages
    find_package(qyh_lift_msgs REQUIRED)
    find_package(qyh_waist_msgs REQUIRED)
    find_package(qyh_gripper_msgs REQUIRED)
    find_package(qyh_standard_robot_msgs REQUIRED)
    find_package(qyh_jaka_control_msgs REQUIRED) # Add JAKA support
    find_package(qyh_task_engine_msgs REQUIRED)
    
else()
    message(FATAL_ERROR "ROS2 dependencies not found. Please source your ROS2 workspace.")
endif()

# ==================== Protobuf 生成 ====================

set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../shared/proto)
set(PROTO_FILES
    ${PROTO_DIR}/common.proto
    ${PROTO_DIR}/control.proto
    ${PROTO_DIR}/state.proto
    ${PROTO_DIR}/messages.proto
)

# 生成 Protobuf 文件
set(PROTO_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR})

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Protobuf 库
add_library(proto_lib STATIC ${PROTO_SRCS})
target_include_directories(proto_lib PUBLIC ${PROTO_GENERATED_DIR} ${Protobuf_INCLUDE_DIRS})
target_link_libraries(proto_lib PUBLIC ${Protobuf_LIBRARIES})

# ==================== 主程序 ====================

set(SOURCES
    src/main.cpp
    src/server.cpp
    src/session.cpp
    src/auth.cpp
    src/message_handler.cpp
    src/state_cache.cpp
    src/config.cpp
    src/rate_limiter.cpp
    src/control_sync.cpp
    src/connection_manager.cpp
    src/ros2_bridge.cpp
    src/watchdog.cpp
    src/vr_session.cpp
)

add_executable(data_plane_server ${SOURCES})

target_include_directories(data_plane_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROTO_GENERATED_DIR}
)

target_link_libraries(data_plane_server
    proto_lib
    Boost::system
    Boost::thread
    Threads::Threads
    yaml-cpp
    OpenSSL::Crypto
    nlohmann_json::nlohmann_json
)

target_compile_definitions(data_plane_server PRIVATE)
ament_target_dependencies(data_plane_server
    rclcpp
    std_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
    nav_msgs
    tf2_msgs
    qyh_lift_msgs
    qyh_waist_msgs
    qyh_gripper_msgs
    qyh_standard_robot_msgs
    qyh_jaka_control_msgs
    qyh_task_engine_msgs
)

# ==================== 安装 ====================

install(TARGETS data_plane_server
    RUNTIME DESTINATION bin
)

install(DIRECTORY config/
    DESTINATION etc/qyh-robot/data_plane
)

# ==================== ROS2 包配置 ====================

if(ament_cmake_FOUND)
    ament_package()
endif()
