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

struct Config {
    RobotConfig robot;
    SignalingConfig signaling;
    WebRtcConfig webrtc;
};

Config load_config(const std::string& path);

} // namespace qyh::robot
