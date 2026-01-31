#include "ros2_control_state_impl.hpp"

namespace qyh::robot::detail {

void publish_robot_state(Ros2ControlStateBridge::Impl& impl,
                         const builtin_interfaces::msg::Time& stamp,
                         const std::string& frame) {
#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    if (!impl.on_state) return;

    qyh::dataplane::StateChannelMessage out;
    out.set_timestamp(static_cast<uint64_t>(stamp.sec) * 1000ULL
                      + static_cast<uint64_t>(stamp.nanosec) / 1000000ULL);
    auto* robot = out.mutable_robot_state();
    fill_header(robot->mutable_header(), frame, stamp);

    if (impl.has_arm) {
        *robot->mutable_arm() = impl.cached_arm;
    }
    if (impl.has_chassis) {
        *robot->mutable_chassis() = impl.cached_chassis;
    }
    if (impl.has_joints) {
        *robot->mutable_joints() = impl.cached_joints;
    }
    if (impl.has_lift) {
        *robot->mutable_lift() = impl.cached_lift;
    }
    if (impl.has_waist) {
        *robot->mutable_waist() = impl.cached_waist;
    }
    if (impl.has_head_pan) {
        *robot->mutable_head_pan() = impl.cached_head_pan;
    }
    if (impl.has_head_tilt) {
        *robot->mutable_head_tilt() = impl.cached_head_tilt;
    }
    if (impl.has_left_gripper) {
        *robot->mutable_left_gripper() = impl.cached_left_gripper;
    }
    if (impl.has_right_gripper) {
        *robot->mutable_right_gripper() = impl.cached_right_gripper;
    }

    robot->set_mode(impl.is_auto_mode ? qyh::dataplane::MODE_AUTO : qyh::dataplane::MODE_TELEOP);
    robot->set_control_held(false);
    robot->set_control_holder("");
    robot->set_system_ready(impl.arm_connected && !impl.chassis_error);
    if (impl.chassis_error) {
        robot->add_active_errors("chassis_error");
        if (impl.last_chassis_error_code != 0) {
            robot->add_active_errors("chassis_error_code=" + std::to_string(impl.last_chassis_error_code));
        }
    }
    if (!impl.left_gripper_connected || !impl.right_gripper_connected) {
        robot->add_warnings("gripper_disconnected");
    }
    if (!impl.lift_connected || !impl.waist_connected) {
        robot->add_warnings("actuator_disconnected");
    }
    if (impl.vr_connected) {
        robot->add_warnings("vr_connected");
    }

    impl.on_state(out);
#else
    (void)impl;
    (void)stamp;
    (void)frame;
#endif
}

} // namespace qyh::robot::detail