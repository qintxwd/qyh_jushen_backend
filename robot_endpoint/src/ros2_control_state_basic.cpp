#include "ros2_control_state_impl.hpp"

namespace qyh::robot::detail {

void publish_basic_state(Ros2ControlStateBridge::Impl& impl,
                         const qyh_standard_robot_msgs::msg::StandardRobotStatus& msg) {
#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    if (!impl.on_state) return;

    qyh::dataplane::StateChannelMessage out;
    const auto& stamp = msg.pose.header.stamp;
    out.set_timestamp(static_cast<uint64_t>(stamp.sec) * 1000ULL
                      + static_cast<uint64_t>(stamp.nanosec) / 1000000ULL);
    auto* basic = out.mutable_basic_state();
    fill_header(basic->mutable_header(), msg.pose.header.frame_id, stamp);

    basic->set_ws_connected(false);
    basic->set_ros_connected(true);

    auto* arm = basic->mutable_arm();
    arm->set_connected(impl.arm_connected);
    arm->set_enabled(impl.arm_connected);
    arm->set_error(false);
    arm->set_error_code(0);

    auto* chassis = basic->mutable_chassis();
    chassis->set_connected(true);
    chassis->set_enabled(msg.can_run_motion_task);
    bool chassis_error = (msg.system_status == msg.SYS_STATUS_ERROR ||
                          msg.system_status == msg.SYS_STATUS_NAV_ERROR ||
                          msg.system_status == msg.SYS_STATUS_HARDWARE_ERROR);
    chassis->set_error(chassis_error);
    chassis->set_error_code(static_cast<int32_t>(msg.last_error_code));

    auto* lift = basic->mutable_lift();
    lift->set_connected(impl.lift_connected);
    lift->set_enabled(impl.lift_connected);
    lift->set_error(false);
    lift->set_error_code(0);

    auto* waist = basic->mutable_waist();
    waist->set_connected(impl.waist_connected);
    waist->set_enabled(impl.waist_connected);
    waist->set_error(false);
    waist->set_error_code(0);

    auto* head = basic->mutable_head();
    head->set_connected(impl.head_connected);
    head->set_enabled(impl.head_connected);
    head->set_error(false);
    head->set_error_code(0);

    auto* camera = basic->mutable_camera();
    camera->set_head_connected(impl.head_connected);
    camera->set_left_hand_connected(false);
    camera->set_right_hand_connected(false);

    auto* gripper = basic->mutable_gripper();
    gripper->set_left_connected(impl.left_gripper_connected);
    gripper->set_left_activated(impl.left_gripper_activated);
    gripper->set_left_fault(0);
    gripper->set_right_connected(impl.right_gripper_connected);
    gripper->set_right_activated(impl.right_gripper_activated);
    gripper->set_right_fault(0);

    basic->set_emergency_stop(msg.is_emergency_stopped);
    auto* battery = basic->mutable_battery();
    battery->set_percentage(msg.battery_remaining_percentage);
    battery->set_voltage(msg.battery_voltage / 1000.0);
    battery->set_charging(msg.is_charging);

    if (msg.is_auto_mode) {
        basic->set_mode(qyh::dataplane::MODE_AUTO);
    } else {
        basic->set_mode(qyh::dataplane::MODE_TELEOP);
    }
    basic->set_control_held(false);
    basic->set_control_holder("");

    basic->set_vr_connected(impl.vr_connected);
    basic->set_vr_left_controller(impl.vr_left_active);
    basic->set_vr_right_controller(impl.vr_right_active);

    impl.on_state(out);
#else
    (void)impl;
    (void)msg;
#endif
}

} // namespace qyh::robot::detail