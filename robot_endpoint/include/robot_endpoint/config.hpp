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
    MediaConfig media;
};

Config load_config(const std::string& path);

} // namespace qyh::robot
