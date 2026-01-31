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
    std::string manual_velocity_topic = "/manual_velocity_cmd";
    std::string manual_motion_topic = "/manual_motion_cmd";
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

    std::string head_command_topic = "/head_motor_node/cmd_position";
    std::string lift_control_service = "/lift/control";
    std::string waist_control_service = "/waist/control";
    std::string left_gripper_activate_service = "/left/activate_gripper";
    std::string left_gripper_move_service = "/left/move_gripper";
    std::string right_gripper_activate_service = "/right/activate_gripper";
    std::string right_gripper_move_service = "/right/move_gripper";

    std::string arm_movej_service = "/jaka/move_j";
    std::string arm_movel_service = "/jaka/move_l";
    std::string arm_jog_service = "/jaka/jog";
    std::string arm_jog_stop_service = "/jaka/jog_stop";

    std::string nav_to_coordinate_service = "/go/navigate_to_coordinate";
    std::string nav_to_site_service = "/go/execute_action_task";
    std::string nav_to_site_task_service = "/go/navigate_to_site_with_task";
    std::string nav_pause_service = "/control/pause_move";
    std::string nav_resume_service = "/control/resume_move";
    std::string nav_cancel_service = "/control/stop_move";

    std::string chassis_emergency_stop_service = "/control/emergency_stop";
    std::string chassis_release_emergency_service = "/control/release_emergency_stop";
    std::string chassis_start_charging_service = "/control/start_charging";
    std::string chassis_stop_charging_service = "/control/stop_charging";
    std::string chassis_enter_low_power_service = "/control/enter_low_power_mode";
    std::string chassis_exit_low_power_service = "/control/exit_low_power_mode";
    std::string chassis_start_manual_service = "/control/start_manual_control";
    std::string chassis_stop_manual_service = "/control/stop_manual_control";
    std::string chassis_pause_mission_service = "/control/pause_mission";
    std::string chassis_resume_mission_service = "/control/resume_mission";
    std::string chassis_cancel_mission_service = "/control/cancel_mission";
    std::string chassis_stop_localization_service = "/control/stop_localization";
    std::string chassis_system_reset_service = "/control/system_reset";

    std::string task_execute_service = "/task_engine/execute";
    std::string task_pause_service = "/task_engine/pause";
    std::string task_resume_service = "/task_engine/resume";
    std::string task_cancel_service = "/task_engine/cancel";
    std::string task_status_topic = "/task_engine/status";

    std::string led_color_topic = "/robot_led/set_color";
    std::string led_blink_topic = "/robot_led/blink";

    std::string shutdown_service = "/shutdown";
    std::string shutdown_state_topic = "/shutdown_state";

    std::string vr_head_pose_topic = "/vr/head/pose";
    std::string vr_left_active_topic = "/vr/left_controller/active";
    std::string vr_right_active_topic = "/vr/right_controller/active";
    std::string vr_left_pose_topic = "/vr/left_controller/pose";
    std::string vr_right_pose_topic = "/vr/right_controller/pose";
    std::string vr_left_joy_topic = "/vr/left_controller/joy";
    std::string vr_right_joy_topic = "/vr/right_controller/joy";
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
