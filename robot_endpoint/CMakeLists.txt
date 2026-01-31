cmake_minimum_required(VERSION 3.16)
project(qyh_robot_endpoint VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(CURL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)
find_package(Protobuf REQUIRED)

option(ROBOT_ENDPOINT_ENABLE_ROS2 "Enable ROS2 integration" OFF)
option(ROBOT_ENDPOINT_ENABLE_WEBRTC "Enable libdatachannel" OFF)
if(ROBOT_ENDPOINT_ENABLE_ROS2)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(qyh_lift_msgs REQUIRED)
    find_package(qyh_waist_msgs REQUIRED)
    find_package(qyh_gripper_msgs REQUIRED)
    find_package(qyh_jaka_control_msgs REQUIRED)
    find_package(qyh_standard_robot_msgs REQUIRED)
    find_package(qyh_task_engine_msgs REQUIRED)
    find_package(qyh_shutdown_msgs REQUIRED)
    add_definitions(-DROBOT_ENDPOINT_ENABLE_ROS2)
endif()

if(ROBOT_ENDPOINT_ENABLE_WEBRTC)
    find_package(LibDataChannel REQUIRED)
    add_definitions(-DROBOT_ENDPOINT_ENABLE_WEBRTC)
endif()
find_package(nlohmann_json 3.9 QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.2
    )
    FetchContent_MakeAvailable(json)
endif()

set(PROTO_DIR ${CMAKE_SOURCE_DIR}/../shared/proto)
file(GLOB PROTO_FILES
    "${PROTO_DIR}/*.proto"
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/signaling_client.cpp
    src/webrtc_peer.cpp
    src/webrtc_session.cpp
    src/data_channel_manager.cpp
    src/config.cpp
    src/media_pipeline.cpp
    src/ros2_bridge.cpp
    src/ros2_control_state.cpp
    src/ros2_control_state_control.cpp
    src/ros2_control_state_state.cpp
    src/ros2_control_state_basic.cpp
    src/ros2_control_state_robot.cpp
    ${PROTO_SRCS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GST_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    CURL::libcurl
    nlohmann_json::nlohmann_json
    yaml-cpp
    ${GST_LIBRARIES}
    protobuf::libprotobuf
)

if(ROBOT_ENDPOINT_ENABLE_ROS2)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        rclcpp::rclcpp
        sensor_msgs::sensor_msgs
        geometry_msgs::geometry_msgs
        nav_msgs::nav_msgs
        std_msgs::std_msgs
        std_srvs::std_srvs
        qyh_lift_msgs::qyh_lift_msgs
        qyh_waist_msgs::qyh_waist_msgs
        qyh_gripper_msgs::qyh_gripper_msgs
        qyh_jaka_control_msgs::qyh_jaka_control_msgs
        qyh_standard_robot_msgs::qyh_standard_robot_msgs
        qyh_task_engine_msgs::qyh_task_engine_msgs
        qyh_shutdown_msgs::qyh_shutdown_msgs
    )
endif()

if(ROBOT_ENDPOINT_ENABLE_WEBRTC)
    target_link_libraries(${PROJECT_NAME} PRIVATE LibDataChannel::LibDataChannel)
endif()

target_compile_options(${PROJECT_NAME} PRIVATE ${GST_CFLAGS_OTHER})
