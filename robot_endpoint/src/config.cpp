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
        cfg.ros2.manual_velocity_topic = root["ros2"]["manual_velocity_topic"].as<std::string>(cfg.ros2.manual_velocity_topic);
        cfg.ros2.manual_motion_topic = root["ros2"]["manual_motion_topic"].as<std::string>(cfg.ros2.manual_motion_topic);
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

        cfg.ros2.head_command_topic = root["ros2"]["head_command_topic"].as<std::string>(cfg.ros2.head_command_topic);
        cfg.ros2.lift_control_service = root["ros2"]["lift_control_service"].as<std::string>(cfg.ros2.lift_control_service);
        cfg.ros2.waist_control_service = root["ros2"]["waist_control_service"].as<std::string>(cfg.ros2.waist_control_service);
        cfg.ros2.left_gripper_activate_service = root["ros2"]["left_gripper_activate_service"].as<std::string>(cfg.ros2.left_gripper_activate_service);
        cfg.ros2.left_gripper_move_service = root["ros2"]["left_gripper_move_service"].as<std::string>(cfg.ros2.left_gripper_move_service);
        cfg.ros2.right_gripper_activate_service = root["ros2"]["right_gripper_activate_service"].as<std::string>(cfg.ros2.right_gripper_activate_service);
        cfg.ros2.right_gripper_move_service = root["ros2"]["right_gripper_move_service"].as<std::string>(cfg.ros2.right_gripper_move_service);
        cfg.ros2.arm_movej_service = root["ros2"]["arm_movej_service"].as<std::string>(cfg.ros2.arm_movej_service);
        cfg.ros2.arm_movel_service = root["ros2"]["arm_movel_service"].as<std::string>(cfg.ros2.arm_movel_service);
        cfg.ros2.arm_jog_service = root["ros2"]["arm_jog_service"].as<std::string>(cfg.ros2.arm_jog_service);
        cfg.ros2.arm_jog_stop_service = root["ros2"]["arm_jog_stop_service"].as<std::string>(cfg.ros2.arm_jog_stop_service);
        cfg.ros2.nav_to_coordinate_service = root["ros2"]["nav_to_coordinate_service"].as<std::string>(cfg.ros2.nav_to_coordinate_service);
        cfg.ros2.nav_to_site_service = root["ros2"]["nav_to_site_service"].as<std::string>(cfg.ros2.nav_to_site_service);
        cfg.ros2.nav_to_site_task_service = root["ros2"]["nav_to_site_task_service"].as<std::string>(cfg.ros2.nav_to_site_task_service);
        cfg.ros2.nav_pause_service = root["ros2"]["nav_pause_service"].as<std::string>(cfg.ros2.nav_pause_service);
        cfg.ros2.nav_resume_service = root["ros2"]["nav_resume_service"].as<std::string>(cfg.ros2.nav_resume_service);
        cfg.ros2.nav_cancel_service = root["ros2"]["nav_cancel_service"].as<std::string>(cfg.ros2.nav_cancel_service);

        cfg.ros2.chassis_emergency_stop_service = root["ros2"]["chassis_emergency_stop_service"].as<std::string>(cfg.ros2.chassis_emergency_stop_service);
        cfg.ros2.chassis_release_emergency_service = root["ros2"]["chassis_release_emergency_service"].as<std::string>(cfg.ros2.chassis_release_emergency_service);
        cfg.ros2.chassis_start_charging_service = root["ros2"]["chassis_start_charging_service"].as<std::string>(cfg.ros2.chassis_start_charging_service);
        cfg.ros2.chassis_stop_charging_service = root["ros2"]["chassis_stop_charging_service"].as<std::string>(cfg.ros2.chassis_stop_charging_service);
        cfg.ros2.chassis_enter_low_power_service = root["ros2"]["chassis_enter_low_power_service"].as<std::string>(cfg.ros2.chassis_enter_low_power_service);
        cfg.ros2.chassis_exit_low_power_service = root["ros2"]["chassis_exit_low_power_service"].as<std::string>(cfg.ros2.chassis_exit_low_power_service);
        cfg.ros2.chassis_start_manual_service = root["ros2"]["chassis_start_manual_service"].as<std::string>(cfg.ros2.chassis_start_manual_service);
        cfg.ros2.chassis_stop_manual_service = root["ros2"]["chassis_stop_manual_service"].as<std::string>(cfg.ros2.chassis_stop_manual_service);
        cfg.ros2.chassis_pause_mission_service = root["ros2"]["chassis_pause_mission_service"].as<std::string>(cfg.ros2.chassis_pause_mission_service);
        cfg.ros2.chassis_resume_mission_service = root["ros2"]["chassis_resume_mission_service"].as<std::string>(cfg.ros2.chassis_resume_mission_service);
        cfg.ros2.chassis_cancel_mission_service = root["ros2"]["chassis_cancel_mission_service"].as<std::string>(cfg.ros2.chassis_cancel_mission_service);
        cfg.ros2.chassis_stop_localization_service = root["ros2"]["chassis_stop_localization_service"].as<std::string>(cfg.ros2.chassis_stop_localization_service);
        cfg.ros2.chassis_system_reset_service = root["ros2"]["chassis_system_reset_service"].as<std::string>(cfg.ros2.chassis_system_reset_service);

        cfg.ros2.task_execute_service = root["ros2"]["task_execute_service"].as<std::string>(cfg.ros2.task_execute_service);
        cfg.ros2.task_pause_service = root["ros2"]["task_pause_service"].as<std::string>(cfg.ros2.task_pause_service);
        cfg.ros2.task_resume_service = root["ros2"]["task_resume_service"].as<std::string>(cfg.ros2.task_resume_service);
        cfg.ros2.task_cancel_service = root["ros2"]["task_cancel_service"].as<std::string>(cfg.ros2.task_cancel_service);
        cfg.ros2.task_status_topic = root["ros2"]["task_status_topic"].as<std::string>(cfg.ros2.task_status_topic);

        cfg.ros2.led_color_topic = root["ros2"]["led_color_topic"].as<std::string>(cfg.ros2.led_color_topic);
        cfg.ros2.led_blink_topic = root["ros2"]["led_blink_topic"].as<std::string>(cfg.ros2.led_blink_topic);

        cfg.ros2.shutdown_service = root["ros2"]["shutdown_service"].as<std::string>(cfg.ros2.shutdown_service);
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
