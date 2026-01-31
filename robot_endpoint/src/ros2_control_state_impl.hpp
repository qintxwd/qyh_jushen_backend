#pragma once

#include "robot_endpoint/ros2_control_state.hpp"

#include <memory>

#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <qyh_lift_msgs/msg/lift_state.hpp>
#include <qyh_waist_msgs/msg/waist_state.hpp>
#include <qyh_gripper_msgs/msg/gripper_state.hpp>
#include <qyh_jaka_control_msgs/msg/robot_state.hpp>
#include <qyh_standard_robot_msgs/msg/standard_robot_status.hpp>
#include <qyh_standard_robot_msgs/msg/navigation_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <qyh_lift_msgs/srv/lift_control.hpp>
#include <qyh_waist_msgs/srv/waist_control.hpp>
#include <qyh_gripper_msgs/srv/move_gripper.hpp>
#include <qyh_gripper_msgs/srv/activate_gripper.hpp>
#include <qyh_jaka_control_msgs/srv/move_j.hpp>
#include <qyh_jaka_control_msgs/srv/move_l.hpp>
#include <qyh_jaka_control_msgs/srv/jog.hpp>
#include <qyh_jaka_control_msgs/srv/jog_stop.hpp>
#include <qyh_standard_robot_msgs/msg/manual_velocity_command.hpp>
#include <qyh_standard_robot_msgs/msg/manual_motion_command.hpp>
#include <qyh_standard_robot_msgs/srv/go_navigate_to_coordinate.hpp>
#include <qyh_standard_robot_msgs/srv/go_execute_action_task.hpp>
#include <qyh_standard_robot_msgs/srv/go_navigate_to_site_with_task.hpp>
#include <qyh_standard_robot_msgs/srv/control_pause_move.hpp>
#include <qyh_standard_robot_msgs/srv/control_resume_move.hpp>
#include <qyh_standard_robot_msgs/srv/control_stop_move.hpp>
#include <qyh_standard_robot_msgs/srv/control_emergency_stop.hpp>
#include <qyh_standard_robot_msgs/srv/control_release_emergency_stop.hpp>
#include <qyh_standard_robot_msgs/srv/control_start_charging.hpp>
#include <qyh_standard_robot_msgs/srv/control_stop_charging.hpp>
#include <qyh_standard_robot_msgs/srv/control_enter_low_power_mode.hpp>
#include <qyh_standard_robot_msgs/srv/control_exit_low_power_mode.hpp>
#include <qyh_standard_robot_msgs/srv/control_start_manual_control.hpp>
#include <qyh_standard_robot_msgs/srv/control_stop_manual_control.hpp>
#include <qyh_standard_robot_msgs/srv/control_pause_mission.hpp>
#include <qyh_standard_robot_msgs/srv/control_resume_mission.hpp>
#include <qyh_standard_robot_msgs/srv/control_cancel_mission.hpp>
#include <qyh_standard_robot_msgs/srv/control_stop_localization.hpp>
#include <qyh_standard_robot_msgs/srv/control_system_reset.hpp>
#include <qyh_task_engine_msgs/srv/execute_task.hpp>
#include <qyh_task_engine_msgs/srv/pause_task.hpp>
#include <qyh_task_engine_msgs/srv/resume_task.hpp>
#include <qyh_task_engine_msgs/srv/cancel_task.hpp>
#include <qyh_task_engine_msgs/msg/task_status.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <std_srvs/srv/trigger.hpp>
#include <qyh_shutdown_msgs/msg/shutdown_state.hpp>
#include <cmath>
#include <array>
#endif

namespace qyh::robot {

#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
inline void fill_timestamp(qyh::dataplane::Timestamp* stamp, const builtin_interfaces::msg::Time& t) {
    stamp->set_seconds(static_cast<int64_t>(t.sec));
    stamp->set_nanos(static_cast<int32_t>(t.nanosec));
}

inline void fill_header(qyh::dataplane::Header* header, const std::string& frame_id,
                        const builtin_interfaces::msg::Time& t) {
    fill_timestamp(header->mutable_stamp(), t);
    header->set_frame_id(frame_id);
    header->set_sequence(0);
}

inline void fill_vector3(qyh::dataplane::Vector3* vec, const geometry_msgs::msg::Vector3& src) {
    vec->set_x(src.x);
    vec->set_y(src.y);
    vec->set_z(src.z);
}

inline void fill_quat(qyh::dataplane::Quaternion* q, const geometry_msgs::msg::Quaternion& src) {
    q->set_x(src.x);
    q->set_y(src.y);
    q->set_z(src.z);
    q->set_w(src.w);
}

inline void fill_pose(qyh::dataplane::Pose* pose, const geometry_msgs::msg::Pose& src) {
    fill_vector3(pose->mutable_position(), src.position);
    fill_quat(pose->mutable_orientation(), src.orientation);
}

inline void fill_twist(qyh::dataplane::Twist* twist, const geometry_msgs::msg::Twist& src) {
    fill_vector3(twist->mutable_linear(), src.linear);
    fill_vector3(twist->mutable_angular(), src.angular);
}

inline double yaw_from_quat(const qyh::dataplane::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny_cosp, cosy_cosp);
}
#endif

struct Ros2ControlStateBridge::Impl {
    Ros2Config cfg;
    StateCallback on_state;

#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<std::thread> spin_thread;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<qyh_standard_robot_msgs::msg::ManualVelocityCommand>::SharedPtr manual_vel_pub;
    rclcpp::Publisher<qyh_standard_robot_msgs::msg::ManualMotionCommand>::SharedPtr manual_motion_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_left_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_right_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_cmd_pub;

    rclcpp::Client<qyh_lift_msgs::srv::LiftControl>::SharedPtr lift_client;
    rclcpp::Client<qyh_waist_msgs::srv::WaistControl>::SharedPtr waist_client;
    rclcpp::Client<qyh_gripper_msgs::srv::ActivateGripper>::SharedPtr left_gripper_activate_client;
    rclcpp::Client<qyh_gripper_msgs::srv::ActivateGripper>::SharedPtr right_gripper_activate_client;
    rclcpp::Client<qyh_gripper_msgs::srv::MoveGripper>::SharedPtr left_gripper_move_client;
    rclcpp::Client<qyh_gripper_msgs::srv::MoveGripper>::SharedPtr right_gripper_move_client;
    rclcpp::Client<qyh_jaka_control_msgs::srv::MoveJ>::SharedPtr arm_movej_client;
    rclcpp::Client<qyh_jaka_control_msgs::srv::MoveL>::SharedPtr arm_movel_client;
    rclcpp::Client<qyh_jaka_control_msgs::srv::Jog>::SharedPtr arm_jog_client;
    rclcpp::Client<qyh_jaka_control_msgs::srv::JogStop>::SharedPtr arm_jog_stop_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::GoNavigateToCoordinate>::SharedPtr nav_coord_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::GoExecuteActionTask>::SharedPtr nav_site_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::GoNavigateToSiteWithTask>::SharedPtr nav_site_task_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlPauseMove>::SharedPtr nav_pause_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlResumeMove>::SharedPtr nav_resume_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlStopMove>::SharedPtr nav_cancel_client;

    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlEmergencyStop>::SharedPtr chassis_emergency_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlReleaseEmergencyStop>::SharedPtr chassis_release_emergency_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlStartCharging>::SharedPtr chassis_start_charging_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlStopCharging>::SharedPtr chassis_stop_charging_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlEnterLowPowerMode>::SharedPtr chassis_enter_low_power_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlExitLowPowerMode>::SharedPtr chassis_exit_low_power_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlStartManualControl>::SharedPtr chassis_start_manual_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlStopManualControl>::SharedPtr chassis_stop_manual_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlPauseMission>::SharedPtr chassis_pause_mission_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlResumeMission>::SharedPtr chassis_resume_mission_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlCancelMission>::SharedPtr chassis_cancel_mission_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlStopLocalization>::SharedPtr chassis_stop_localization_client;
    rclcpp::Client<qyh_standard_robot_msgs::srv::ControlSystemReset>::SharedPtr chassis_system_reset_client;

    rclcpp::Client<qyh_task_engine_msgs::srv::ExecuteTask>::SharedPtr task_execute_client;
    rclcpp::Client<qyh_task_engine_msgs::srv::PauseTask>::SharedPtr task_pause_client;
    rclcpp::Client<qyh_task_engine_msgs::srv::ResumeTask>::SharedPtr task_resume_client;
    rclcpp::Client<qyh_task_engine_msgs::srv::CancelTask>::SharedPtr task_cancel_client;

    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr led_color_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr led_blink_pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_client;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<qyh_standard_robot_msgs::msg::StandardRobotStatus>::SharedPtr chassis_sub;
    rclcpp::Subscription<qyh_standard_robot_msgs::msg::NavigationStatus>::SharedPtr nav_sub;
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::RobotState>::SharedPtr arm_state_sub;
    rclcpp::Subscription<qyh_lift_msgs::msg::LiftState>::SharedPtr lift_state_sub;
    rclcpp::Subscription<qyh_waist_msgs::msg::WaistState>::SharedPtr waist_state_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr head_state_sub;
    rclcpp::Subscription<qyh_gripper_msgs::msg::GripperState>::SharedPtr left_gripper_sub;
    rclcpp::Subscription<qyh_gripper_msgs::msg::GripperState>::SharedPtr right_gripper_sub;
    rclcpp::Subscription<qyh_task_engine_msgs::msg::TaskStatus>::SharedPtr task_status_sub;
    rclcpp::Subscription<qyh_shutdown_msgs::msg::ShutdownState>::SharedPtr shutdown_state_sub;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vr_head_pose_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vr_left_active_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vr_right_active_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vr_left_pose_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vr_right_pose_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr vr_left_joy_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr vr_right_joy_sub;

    qyh::dataplane::Pose vr_head_pose;
    qyh::dataplane::Pose vr_left_pose;
    qyh::dataplane::Pose vr_right_pose;
    bool vr_connected = false;
    bool vr_left_active = false;
    bool vr_right_active = false;
    bool vr_left_clutch = false;
    bool vr_right_clutch = false;

    bool arm_connected = false;
    bool lift_connected = false;
    bool waist_connected = false;
    bool head_connected = false;
    bool left_gripper_connected = false;
    bool right_gripper_connected = false;
    bool left_gripper_activated = false;
    bool right_gripper_activated = false;

    bool has_arm = false;
    bool has_chassis = false;
    bool has_joints = false;
    bool has_lift = false;
    bool has_waist = false;
    bool has_head_pan = false;
    bool has_head_tilt = false;
    bool has_left_gripper = false;
    bool has_right_gripper = false;
    bool is_auto_mode = false;
    bool chassis_error = false;
    uint32_t last_chassis_error_code = 0;

    qyh::dataplane::ArmState cached_arm;
    qyh::dataplane::ChassisState cached_chassis;
    qyh::dataplane::JointState cached_joints;
    qyh::dataplane::ActuatorState cached_lift;
    qyh::dataplane::ActuatorState cached_waist;
    qyh::dataplane::ActuatorState cached_head_pan;
    qyh::dataplane::ActuatorState cached_head_tilt;
    qyh::dataplane::GripperState cached_left_gripper;
    qyh::dataplane::GripperState cached_right_gripper;

    qyh_standard_robot_msgs::msg::NavigationStatus last_nav;
    bool has_nav = false;
#endif
};

namespace detail {
    void setup_control_io(Ros2ControlStateBridge::Impl& impl);
    void setup_state_subscriptions(Ros2ControlStateBridge::Impl& impl);
    void publish_basic_state(Ros2ControlStateBridge::Impl& impl,
                             const qyh_standard_robot_msgs::msg::StandardRobotStatus& msg);
    void publish_robot_state(Ros2ControlStateBridge::Impl& impl,
                             const builtin_interfaces::msg::Time& stamp,
                             const std::string& frame);
}

} // namespace qyh::robot