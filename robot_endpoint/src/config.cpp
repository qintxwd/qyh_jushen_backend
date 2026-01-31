#include "robot_endpoint/config.hpp"

#include <yaml-cpp/yaml.h>

namespace qyh::robot {

Config load_config(const std::string& path) {
    Config cfg;

    YAML::Node root = YAML::LoadFile(path);
    if (root["robot"]) {
        cfg.robot.id = root["robot"]["id"].as<std::string>(cfg.robot.id);
        cfg.robot.name = root["robot"]["name"].as<std::string>(cfg.robot.name);
    }
    if (root["signaling"]) {
        cfg.signaling.server_url = root["signaling"]["server_url"].as<std::string>(cfg.signaling.server_url);
        cfg.signaling.robot_secret = root["signaling"]["robot_secret"].as<std::string>(cfg.signaling.robot_secret);
    }
    if (root["webrtc"] && root["webrtc"]["ice_servers"]) {
        for (const auto& node : root["webrtc"]["ice_servers"]) {
            IceServer s;
            s.urls = node["urls"].as<std::string>("");
            s.username = node["username"].as<std::string>("");
            s.credential = node["credential"].as<std::string>("");
            cfg.webrtc.ice_servers.push_back(s);
        }
    }

    if (root["media"]) {
        cfg.media.encoding = root["media"]["encoding"].as<std::string>(cfg.media.encoding);
        if (root["media"]["cameras"]) {
            for (const auto& node : root["media"]["cameras"]) {
                CameraConfig cam;
                cam.name = node["name"].as<std::string>("");
                cam.color_topic = node["color_topic"].as<std::string>("");
                cam.depth_topic = node["depth_topic"].as<std::string>("");
                cam.width = node["width"].as<int>(cam.width);
                cam.height = node["height"].as<int>(cam.height);
                cam.fps = node["fps"].as<int>(cam.fps);
                cfg.media.cameras.push_back(cam);
            }
        }
    }

    return cfg;
}

} // namespace qyh::robot
