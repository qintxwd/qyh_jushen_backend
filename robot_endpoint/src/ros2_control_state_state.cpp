#include "ros2_control_state_impl.hpp"

#include <iostream>

namespace qyh::robot {

namespace detail {
void setup_state_subscriptions(Ros2ControlStateBridge::Impl& impl) {
#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    if (!impl.cfg.joint_state_topic.empty()) {
        impl.joint_state_sub = impl.node->create_subscription<sensor_msgs::msg::JointState>(
            impl.cfg.joint_state_topic, rclcpp::SensorDataQoS(),
            [&impl](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                if (!impl.on_state) return;
                qyh::dataplane::StateChannelMessage out;
                out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                  + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                auto* state = out.mutable_joint_state();
                fill_header(state->mutable_header(), msg->header.frame_id, msg->header.stamp);
                for (const auto& n : msg->name) {
                    state->add_names(n);
                }
                for (const auto& v : msg->position) {
                    state->add_positions(v);
                }
                for (const auto& v : msg->velocity) {
                    state->add_velocities(v);
                }
                for (const auto& v : msg->effort) {
                    state->add_efforts(v);
                }
                impl.on_state(out);
            });
    }

    if (!impl.cfg.imu_topic.empty()) {
        impl.imu_sub = impl.node->create_subscription<sensor_msgs::msg::Imu>(
            impl.cfg.imu_topic, rclcpp::SensorDataQoS(),
            [&impl](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                if (!impl.on_state) return;
                qyh::dataplane::StateChannelMessage out;
                out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                  + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                auto* imu = out.mutable_imu();
                fill_header(imu->mutable_header(), msg->header.frame_id, msg->header.stamp);
                fill_quat(imu->mutable_orientation(), msg->orientation);
                fill_vector3(imu->mutable_angular_velocity(), msg->angular_velocity);
                fill_vector3(imu->mutable_linear_acceleration(), msg->linear_acceleration);
                impl.on_state(out);
            });
    }

    if (!impl.cfg.odom_topic.empty()) {
        impl.odom_sub = impl.node->create_subscription<nav_msgs::msg::Odometry>(
            impl.cfg.odom_topic, rclcpp::SensorDataQoS(),
            [&impl](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                if (!impl.on_state) return;
                qyh::dataplane::StateChannelMessage out;
                out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                  + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                auto* chassis = out.mutable_chassis_state();
                fill_header(chassis->mutable_header(), msg->header.frame_id, msg->header.stamp);
                fill_pose(chassis->mutable_odom(), msg->pose.pose);
                fill_twist(chassis->mutable_velocity(), msg->twist.twist);
                impl.on_state(out);
            });
    }

    if (!impl.cfg.navigation_status_topic.empty()) {
        impl.nav_sub = impl.node->create_subscription<qyh_standard_robot_msgs::msg::NavigationStatus>(
            impl.cfg.navigation_status_topic, rclcpp::SensorDataQoS(),
            [&impl](qyh_standard_robot_msgs::msg::NavigationStatus::ConstSharedPtr msg) {
                impl.last_nav = *msg;
                impl.has_nav = true;
            });
    }

    if (!impl.cfg.standard_robot_status_topic.empty()) {
        impl.chassis_sub = impl.node->create_subscription<qyh_standard_robot_msgs::msg::StandardRobotStatus>(
            impl.cfg.standard_robot_status_topic, rclcpp::SensorDataQoS(),
            [&impl](qyh_standard_robot_msgs::msg::StandardRobotStatus::ConstSharedPtr msg) {
                if (!impl.on_state) return;

                qyh::dataplane::StateChannelMessage out;
                const auto& stamp = msg->pose.header.stamp;
                out.set_timestamp(static_cast<uint64_t>(stamp.sec) * 1000ULL
                                  + static_cast<uint64_t>(stamp.nanosec) / 1000000ULL);
                auto* chassis = out.mutable_chassis_state();
                fill_header(chassis->mutable_header(), msg->pose.header.frame_id, stamp);
                fill_pose(chassis->mutable_odom(), msg->pose.pose.pose);
                fill_twist(chassis->mutable_velocity(), msg->twist);
                chassis->set_battery_level(msg->battery_remaining_percentage);
                chassis->set_charging(msg->is_charging);
                chassis->set_emergency_stop(msg->is_emergency_stopped);

                auto* nav = chassis->mutable_navigation();
                if (msg->system_status == msg->SYS_STATUS_NAV_PATHFINDING ||
                    msg->system_status == msg->SYS_STATUS_WAITING_ARRIVAL ||
                    msg->system_status == msg->SYS_STATUS_NAV_REPATHING ||
                    msg->system_status == msg->SYS_STATUS_FIXED_PATH_INITIALIZING ||
                    msg->system_status == msg->SYS_STATUS_WAITING_FIXED_PATH_END ||
                    msg->system_status == msg->SYS_STATUS_FIXED_PATH_OBSTACLE_DETECTED) {
                    nav->set_state(qyh::dataplane::NavigationStatus::NAVIGATING);
                } else if (msg->system_status == msg->SYS_STATUS_OBSTACLE_PAUSED ||
                           msg->system_status == msg->SYS_STATUS_USER_PAUSED_FIXED_PATH) {
                    nav->set_state(qyh::dataplane::NavigationStatus::PAUSED);
                } else if (msg->system_status == msg->SYS_STATUS_CANNOT_ARRIVE ||
                           msg->system_status == msg->SYS_STATUS_NAV_ERROR ||
                           msg->system_status == msg->SYS_STATUS_HARDWARE_ERROR) {
                    nav->set_state(qyh::dataplane::NavigationStatus::FAILED);
                    nav->set_error_message("navigation_error");
                } else {
                    nav->set_state(qyh::dataplane::NavigationStatus::IDLE);
                }

                if (impl.has_nav) {
                    const auto& n = impl.last_nav;
                    qyh::dataplane::Pose* goal = nav->mutable_current_goal();
                    goal->mutable_position()->set_x(n.autonomous_nav_pose.x);
                    goal->mutable_position()->set_y(n.autonomous_nav_pose.y);
                    goal->mutable_position()->set_z(0.0);
                }

                impl.on_state(out);
            });
    }

    if (!impl.cfg.arm_state_topic.empty()) {
        impl.arm_state_sub = impl.node->create_subscription<qyh_jaka_control_msgs::msg::RobotState>(
            impl.cfg.arm_state_topic, rclcpp::SensorDataQoS(),
            [&impl](qyh_jaka_control_msgs::msg::RobotState::ConstSharedPtr msg) {
                if (!impl.on_state) return;
                qyh::dataplane::StateChannelMessage out;
                out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                  + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                auto* robot = out.mutable_robot_state();
                fill_header(robot->mutable_header(), msg->header.frame_id, msg->header.stamp);
                auto* arm = robot->mutable_arm();
                fill_header(arm->mutable_header(), msg->header.frame_id, msg->header.stamp);
                arm->set_connected(msg->connected);
                arm->set_powered_on(msg->powered_on);
                arm->set_enabled(msg->enabled);
                arm->set_in_estop(msg->in_estop);
                arm->set_in_error(msg->in_error);
                arm->set_servo_mode(msg->servo_mode_enabled);
                arm->set_error_message(msg->error_message);
                arm->set_left_in_position(msg->left_in_position);
                arm->set_right_in_position(msg->right_in_position);
                for (const auto& v : msg->left_joint_positions) {
                    arm->add_left_positions(v);
                }
                for (const auto& v : msg->right_joint_positions) {
                    arm->add_right_positions(v);
                }
                fill_pose(arm->mutable_left_end_effector(), msg->left_cartesian_pose);
                fill_pose(arm->mutable_right_end_effector(), msg->right_cartesian_pose);
                impl.on_state(out);
            });
    }

    if (!impl.cfg.lift_state_topic.empty()) {
        impl.lift_state_sub = impl.node->create_subscription<qyh_lift_msgs::msg::LiftState>(
            impl.cfg.lift_state_topic, rclcpp::SensorDataQoS(),
            [&impl](qyh_lift_msgs::msg::LiftState::ConstSharedPtr msg) {
                if (!impl.on_state) return;
                qyh::dataplane::StateChannelMessage out;
                out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                  + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                auto* act = out.mutable_actuator_state();
                fill_header(act->mutable_header(), msg->header.frame_id, msg->header.stamp);
                act->set_actuator_id("lift");
                act->set_position(msg->current_position);
                act->set_velocity(msg->current_speed);
                act->set_in_motion(!msg->position_reached);
                act->set_in_position(msg->position_reached);
                impl.on_state(out);
            });
    }

    if (!impl.cfg.waist_state_topic.empty()) {
        impl.waist_state_sub = impl.node->create_subscription<qyh_waist_msgs::msg::WaistState>(
            impl.cfg.waist_state_topic, rclcpp::SensorDataQoS(),
            [&impl](qyh_waist_msgs::msg::WaistState::ConstSharedPtr msg) {
                if (!impl.on_state) return;
                qyh::dataplane::StateChannelMessage out;
                out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                  + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                auto* act = out.mutable_actuator_state();
                fill_header(act->mutable_header(), msg->header.frame_id, msg->header.stamp);
                act->set_actuator_id("waist");
                act->set_position(msg->current_angle);
                act->set_velocity(static_cast<double>(msg->current_speed));
                act->set_in_motion(!msg->position_reached);
                act->set_in_position(msg->position_reached);
                impl.on_state(out);
            });
    }

    if (!impl.cfg.head_joint_state_topic.empty()) {
        impl.head_state_sub = impl.node->create_subscription<sensor_msgs::msg::JointState>(
            impl.cfg.head_joint_state_topic, rclcpp::SensorDataQoS(),
            [&impl](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                if (!impl.on_state) return;
                for (size_t i = 0; i < msg->name.size(); ++i) {
                    const auto& name = msg->name[i];
                    double pos = (i < msg->position.size()) ? msg->position[i] : 0.0;
                    double vel = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
                    if (name.find("pan") == std::string::npos && name.find("tilt") == std::string::npos) {
                        continue;
                    }
                    qyh::dataplane::StateChannelMessage out;
                    out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                      + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                    auto* act = out.mutable_actuator_state();
                    fill_header(act->mutable_header(), msg->header.frame_id, msg->header.stamp);
                    act->set_actuator_id(name.find("pan") != std::string::npos ? "head_pan" : "head_tilt");
                    act->set_position(pos);
                    act->set_velocity(vel);
                    act->set_in_motion(std::abs(vel) > 1e-6);
                    act->set_in_position(std::abs(vel) <= 1e-6);
                    impl.on_state(out);
                }
            });
    }

    if (!impl.cfg.left_gripper_state_topic.empty()) {
        impl.left_gripper_sub = impl.node->create_subscription<qyh_gripper_msgs::msg::GripperState>(
            impl.cfg.left_gripper_state_topic, rclcpp::SensorDataQoS(),
            [&impl](qyh_gripper_msgs::msg::GripperState::ConstSharedPtr msg) {
                if (!impl.on_state) return;
                qyh::dataplane::StateChannelMessage out;
                out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                  + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                auto* g = out.mutable_gripper_state();
                fill_header(g->mutable_header(), msg->header.frame_id, msg->header.stamp);
                g->set_gripper_id("left");
                g->set_position(static_cast<double>(msg->current_position) / 255.0);
                g->set_force(static_cast<double>(msg->current_force));
                g->set_object_detected(msg->object_status == 2);
                g->set_in_motion(msg->is_moving);
                impl.on_state(out);
            });
    }

    if (!impl.cfg.right_gripper_state_topic.empty()) {
        impl.right_gripper_sub = impl.node->create_subscription<qyh_gripper_msgs::msg::GripperState>(
            impl.cfg.right_gripper_state_topic, rclcpp::SensorDataQoS(),
            [&impl](qyh_gripper_msgs::msg::GripperState::ConstSharedPtr msg) {
                if (!impl.on_state) return;
                qyh::dataplane::StateChannelMessage out;
                out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                  + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                auto* g = out.mutable_gripper_state();
                fill_header(g->mutable_header(), msg->header.frame_id, msg->header.stamp);
                g->set_gripper_id("right");
                g->set_position(static_cast<double>(msg->current_position) / 255.0);
                g->set_force(static_cast<double>(msg->current_force));
                g->set_object_detected(msg->object_status == 2);
                g->set_in_motion(msg->is_moving);
                impl.on_state(out);
            });
    }

    if (!impl.cfg.task_status_topic.empty()) {
        impl.task_status_sub = impl.node->create_subscription<qyh_task_engine_msgs::msg::TaskStatus>(
            impl.cfg.task_status_topic, rclcpp::SensorDataQoS(),
            [&impl](qyh_task_engine_msgs::msg::TaskStatus::ConstSharedPtr msg) {
                if (!impl.on_state) return;
                qyh::dataplane::StateChannelMessage out;
                out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                  + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                auto* task = out.mutable_task_state();
                fill_header(task->mutable_header(), msg->header.frame_id, msg->header.stamp);
                try {
                    task->set_task_id(std::stoll(msg->task_id));
                } catch (...) {
                    task->set_task_id(0);
                }
                task->set_task_name(msg->task_name);
                if (msg->status == "running") {
                    task->set_status(qyh::dataplane::TaskState::RUNNING);
                } else if (msg->status == "paused") {
                    task->set_status(qyh::dataplane::TaskState::PAUSED);
                } else if (msg->status == "success") {
                    task->set_status(qyh::dataplane::TaskState::COMPLETED);
                } else if (msg->status == "failure") {
                    task->set_status(qyh::dataplane::TaskState::FAILED);
                } else if (msg->status == "cancelled") {
                    task->set_status(qyh::dataplane::TaskState::CANCELLED);
                } else {
                    task->set_status(qyh::dataplane::TaskState::PENDING);
                }
                task->set_current_step(static_cast<int32_t>(msg->completed_nodes));
                task->set_total_steps(static_cast<int32_t>(msg->total_nodes));
                task->set_progress(msg->progress);
                task->set_current_action(msg->current_node_id);
                task->set_error_message(msg->message);
                impl.on_state(out);
            });
    }
#else
    (void)impl;
#endif
}
} // namespace detail

} // namespace qyh::robot