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
find_package(Boost 1.74 REQUIRED COMPONENTS system thread)

# Protobuf
find_package(Protobuf REQUIRED)

# Threads
find_package(Threads REQUIRED)

# yaml-cpp (配置文件解析)
find_package(yaml-cpp REQUIRED)

# ROS2 (可选)
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
    message(STATUS "Building with ROS2 support")
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    set(WITH_ROS2 ON)
else()
    message(STATUS "Building without ROS2 support")
    set(WITH_ROS2 OFF)
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
set(PROTO_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto)
file(MAKE_DIRECTORY ${PROTO_GENERATED_DIR})

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
)

if(WITH_ROS2)
    list(APPEND SOURCES
        src/ros2_bridge.cpp
        src/watchdog.cpp
    )
endif()

add_executable(data_plane_server ${SOURCES})

target_include_directories(data_plane_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROTO_GENERATED_DIR}
)

target_link_libraries(data_plane_server PRIVATE
    proto_lib
    Boost::system
    Boost::thread
    Threads::Threads
    yaml-cpp
)

if(WITH_ROS2)
    target_compile_definitions(data_plane_server PRIVATE WITH_ROS2)
    ament_target_dependencies(data_plane_server
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
    )
endif()

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
