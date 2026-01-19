cmake_minimum_required(VERSION 3.16)
project(qyh_media_plane VERSION 2.0.0 LANGUAGES CXX)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译选项
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wall -Wextra -Wpedantic)

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
find_package(Boost 1.74 REQUIRED COMPONENTS system thread)

# Threads
find_package(Threads REQUIRED)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# JSON (for signaling messages)
find_package(nlohmann_json 3.9 REQUIRED)

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

add_executable(media_plane_server ${SOURCES})

target_include_directories(media_plane_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GST_INCLUDE_DIRS}
)

target_link_libraries(media_plane_server PRIVATE
    ${GST_LIBRARIES}
    Boost::system
    Boost::thread
    Threads::Threads
    yaml-cpp
    nlohmann_json::nlohmann_json
)

target_compile_options(media_plane_server PRIVATE ${GST_CFLAGS_OTHER})

# ==================== 安装 ====================

install(TARGETS media_plane_server
    RUNTIME DESTINATION bin
)

install(DIRECTORY config/
    DESTINATION etc/qyh-robot/media_plane
)
