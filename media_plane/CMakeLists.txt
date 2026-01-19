cmake_minimum_required(VERSION 3.16)
project(qyh_media_plane VERSION 2.0.0 LANGUAGES CXX)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)

# ==================== ROS2 依赖 ====================

# 尝试查找 ament_cmake（ROS2 构建系统）
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
    message(STATUS "ROS2 ament_cmake found, enabling ROS2 image source")
    set(ENABLE_ROS2 ON)
    
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    
    add_definitions(-DENABLE_ROS2)
else()
    message(WARNING "ROS2 not found, ROS2 image source will be disabled")
    set(ENABLE_ROS2 OFF)
endif()

# ==================== 依赖查找 ====================

# GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED 
    gstreamer-1.0>=1.20
    gstreamer-app-1.0
    gstreamer-video-1.0
    gstreamer-webrtc-1.0
    gstreamer-sdp-1.0
)

# Boost (WebSocket signaling)
find_package(Boost REQUIRED COMPONENTS system thread)

# Threads
find_package(Threads REQUIRED)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# JSON (for signaling messages)
find_package(nlohmann_json 3.9 REQUIRED)

# OpenSSL (JWT signature verification)
find_package(OpenSSL REQUIRED)

# ==================== 主程序 ====================

set(SOURCES
    src/main.cpp
    src/media_server.cpp
    src/signaling_server.cpp
    src/webrtc_peer.cpp
    src/pipeline_manager.cpp
    src/config.cpp
    src/auth.cpp
)

# 如果启用 ROS2，添加 ROS2 图像源
if(ENABLE_ROS2)
    list(APPEND SOURCES src/ros2_image_source.cpp)
endif()

add_executable(media_plane_server ${SOURCES})

target_include_directories(media_plane_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GST_INCLUDE_DIRS}
)

target_link_libraries(media_plane_server
    ${GST_LIBRARIES}
    Boost::system
    Boost::thread
    Threads::Threads
    yaml-cpp
    nlohmann_json::nlohmann_json
    OpenSSL::Crypto
)

# ROS2 依赖链接
if(ENABLE_ROS2)
    ament_target_dependencies(media_plane_server
        rclcpp
        sensor_msgs
    )
endif()

target_compile_options(media_plane_server PRIVATE ${GST_CFLAGS_OTHER})

# ==================== 安装 ====================

install(TARGETS media_plane_server
    RUNTIME DESTINATION bin
)

install(DIRECTORY config/
    DESTINATION etc/qyh-robot/media_plane
)

# ROS2 ament 包安装（如果是 ROS2 环境）
if(ENABLE_ROS2)
    ament_package()
endif()
