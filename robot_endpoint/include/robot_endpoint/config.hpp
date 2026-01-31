#pragma once

#include <string>
#include <vector>

namespace qyh::robot {

struct IceServer {
    std::string urls;
    std::string username;
    std::string credential;
};

struct RobotConfig {
    std::string id = "qyh-robot-001";
    std::string name = "QYH Robot";
};

struct SignalingConfig {
    std::string server_url = "http://127.0.0.1:8000/api/v1/signaling";
    std::string robot_secret = "qyh-robot-secret";
};

struct WebRtcConfig {
    std::vector<IceServer> ice_servers;
};

struct Ros2Config {
    std::string cmd_vel_topic = "/cmd_vel";
    std::string joint_command_topic = "/joint_command";
    std::string gripper_command_topic_prefix = "/gripper/command";
    std::string joint_state_topic = "/joint_states";
    std::string imu_topic = "/imu";
    std::string odom_topic = "/odom";
    std::string standard_robot_status_topic = "standard_robot_status";
    std::string navigation_status_topic = "navigation_status";
    std::string arm_state_topic = "/jaka/robot_state";
    std::string head_joint_state_topic = "/head/joint_states";
    std::string lift_state_topic = "/lift/state";
    std::string waist_state_topic = "/waist/state";
    std::string left_gripper_state_topic = "/left/gripper_state";
    std::string right_gripper_state_topic = "/right/gripper_state";
};

struct CameraConfig {
    std::string name;
    std::string color_topic;
    std::string depth_topic;
    int width = 1280;
    int height = 720;
    int fps = 0; // 0 表示跟随 ROS 实际频率
};

struct MediaConfig {
    std::string encoding = "h265";
    std::vector<CameraConfig> cameras;
};

struct Config {
    RobotConfig robot;
    SignalingConfig signaling;
    WebRtcConfig webrtc;
    Ros2Config ros2;
    MediaConfig media;
};

Config load_config(const std::string& path);

} // namespace qyh::robot
