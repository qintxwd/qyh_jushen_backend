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

    if (root["ros2"]) {
        cfg.ros2.cmd_vel_topic = root["ros2"]["cmd_vel_topic"].as<std::string>(cfg.ros2.cmd_vel_topic);
        cfg.ros2.joint_command_topic = root["ros2"]["joint_command_topic"].as<std::string>(cfg.ros2.joint_command_topic);
        cfg.ros2.gripper_command_topic_prefix = root["ros2"]["gripper_command_topic_prefix"].as<std::string>(cfg.ros2.gripper_command_topic_prefix);
        cfg.ros2.joint_state_topic = root["ros2"]["joint_state_topic"].as<std::string>(cfg.ros2.joint_state_topic);
        cfg.ros2.imu_topic = root["ros2"]["imu_topic"].as<std::string>(cfg.ros2.imu_topic);
        cfg.ros2.odom_topic = root["ros2"]["odom_topic"].as<std::string>(cfg.ros2.odom_topic);
        cfg.ros2.standard_robot_status_topic = root["ros2"]["standard_robot_status_topic"].as<std::string>(cfg.ros2.standard_robot_status_topic);
        cfg.ros2.navigation_status_topic = root["ros2"]["navigation_status_topic"].as<std::string>(cfg.ros2.navigation_status_topic);
        cfg.ros2.arm_state_topic = root["ros2"]["arm_state_topic"].as<std::string>(cfg.ros2.arm_state_topic);
        cfg.ros2.head_joint_state_topic = root["ros2"]["head_joint_state_topic"].as<std::string>(cfg.ros2.head_joint_state_topic);
        cfg.ros2.lift_state_topic = root["ros2"]["lift_state_topic"].as<std::string>(cfg.ros2.lift_state_topic);
        cfg.ros2.waist_state_topic = root["ros2"]["waist_state_topic"].as<std::string>(cfg.ros2.waist_state_topic);
        cfg.ros2.left_gripper_state_topic = root["ros2"]["left_gripper_state_topic"].as<std::string>(cfg.ros2.left_gripper_state_topic);
        cfg.ros2.right_gripper_state_topic = root["ros2"]["right_gripper_state_topic"].as<std::string>(cfg.ros2.right_gripper_state_topic);
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
