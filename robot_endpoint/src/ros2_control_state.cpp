#include "robot_endpoint/ros2_control_state.hpp"

#include <iostream>
#include <thread>
#include <utility>

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
#include <cmath>
#endif

namespace qyh::robot {

namespace {
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
#endif
} // namespace

struct Ros2ControlStateBridge::Impl {
    Ros2Config cfg;
    StateCallback on_state;

#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<std::thread> spin_thread;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_left_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_right_pub;

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

    qyh_standard_robot_msgs::msg::NavigationStatus last_nav;
    bool has_nav = false;
#endif
};

Ros2ControlStateBridge::Ros2ControlStateBridge(const Ros2Config& cfg)
    : impl_(std::make_unique<Impl>()) {
    impl_->cfg = cfg;
}

Ros2ControlStateBridge::~Ros2ControlStateBridge() {
    stop();
}

void Ros2ControlStateBridge::set_on_state(StateCallback cb) {
    impl_->on_state = std::move(cb);
}

void Ros2ControlStateBridge::handle_control(const qyh::dataplane::ControlChannelMessage& msg) {
#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    if (!impl_->node) {
        return;
    }
    switch (msg.payload_case()) {
        case qyh::dataplane::ControlChannelMessage::kChassisVel: {
            if (!impl_->cmd_vel_pub) break;
            geometry_msgs::msg::Twist twist;
            const auto& vel = msg.chassis_vel();
            twist.linear.x = vel.linear_x();
            twist.linear.y = vel.linear_y();
            twist.angular.z = vel.angular_z();
            impl_->cmd_vel_pub->publish(twist);
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kJointCmd: {
            if (!impl_->joint_cmd_pub) break;
            sensor_msgs::msg::JointState js;
            const auto& cmd = msg.joint_cmd();
            js.name.assign(cmd.names().begin(), cmd.names().end());
            js.position.assign(cmd.positions().begin(), cmd.positions().end());
            js.velocity.assign(cmd.velocities().begin(), cmd.velocities().end());
            impl_->joint_cmd_pub->publish(js);
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kGripper: {
            const auto& gripper = msg.gripper();
            std_msgs::msg::Float64MultiArray arr;
            arr.data.resize(2);
            arr.data[0] = gripper.position();
            arr.data[1] = gripper.force();
            if (gripper.gripper_id() == "left" && impl_->gripper_left_pub) {
                impl_->gripper_left_pub->publish(arr);
            } else if (gripper.gripper_id() == "right" && impl_->gripper_right_pub) {
                impl_->gripper_right_pub->publish(arr);
            } else {
                std::cout << "[Ros2ControlStateBridge] unknown gripper_id=" << gripper.gripper_id() << std::endl;
            }
            break;
        }
        default:
            std::cout << "[Ros2ControlStateBridge] control payload not mapped: "
                      << msg.payload_case() << std::endl;
            break;
    }
#else
    (void)msg;
#endif
}

void Ros2ControlStateBridge::start() {
#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    if (rclcpp::ok()) {
        impl_->node = std::make_shared<rclcpp::Node>("qyh_robot_endpoint_ctrl");

        impl_->cmd_vel_pub = impl_->node->create_publisher<geometry_msgs::msg::Twist>(
            impl_->cfg.cmd_vel_topic, 10);
        impl_->joint_cmd_pub = impl_->node->create_publisher<sensor_msgs::msg::JointState>(
            impl_->cfg.joint_command_topic, 10);

        impl_->gripper_left_pub = impl_->node->create_publisher<std_msgs::msg::Float64MultiArray>(
            impl_->cfg.gripper_command_topic_prefix + "/left", 10);
        impl_->gripper_right_pub = impl_->node->create_publisher<std_msgs::msg::Float64MultiArray>(
            impl_->cfg.gripper_command_topic_prefix + "/right", 10);

        if (!impl_->cfg.joint_state_topic.empty()) {
            impl_->joint_state_sub = impl_->node->create_subscription<sensor_msgs::msg::JointState>(
                impl_->cfg.joint_state_topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                    if (!impl_->on_state) return;
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
                    impl_->on_state(out);
                });
        }

        if (!impl_->cfg.imu_topic.empty()) {
            impl_->imu_sub = impl_->node->create_subscription<sensor_msgs::msg::Imu>(
                impl_->cfg.imu_topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                    if (!impl_->on_state) return;
                    qyh::dataplane::StateChannelMessage out;
                    out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                      + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                    auto* imu = out.mutable_imu();
                    fill_header(imu->mutable_header(), msg->header.frame_id, msg->header.stamp);
                    fill_quat(imu->mutable_orientation(), msg->orientation);
                    fill_vector3(imu->mutable_angular_velocity(), msg->angular_velocity);
                    fill_vector3(imu->mutable_linear_acceleration(), msg->linear_acceleration);
                    impl_->on_state(out);
                });
        }

        if (!impl_->cfg.odom_topic.empty()) {
            impl_->odom_sub = impl_->node->create_subscription<nav_msgs::msg::Odometry>(
                impl_->cfg.odom_topic, rclcpp::SensorDataQoS(),
                [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                    if (!impl_->on_state) return;
                    qyh::dataplane::StateChannelMessage out;
                    out.set_timestamp(static_cast<uint64_t>(msg->header.stamp.sec) * 1000ULL
                                      + static_cast<uint64_t>(msg->header.stamp.nanosec) / 1000000ULL);
                    auto* chassis = out.mutable_chassis_state();
                    fill_header(chassis->mutable_header(), msg->header.frame_id, msg->header.stamp);
                    fill_pose(chassis->mutable_odom(), msg->pose.pose);
                    fill_twist(chassis->mutable_velocity(), msg->twist.twist);
                    impl_->on_state(out);
                });
        }

        if (!impl_->cfg.navigation_status_topic.empty()) {
            impl_->nav_sub = impl_->node->create_subscription<qyh_standard_robot_msgs::msg::NavigationStatus>(
                impl_->cfg.navigation_status_topic, rclcpp::SensorDataQoS(),
                [this](qyh_standard_robot_msgs::msg::NavigationStatus::ConstSharedPtr msg) {
                    impl_->last_nav = *msg;
                    impl_->has_nav = true;
                });
        }

        if (!impl_->cfg.standard_robot_status_topic.empty()) {
            impl_->chassis_sub = impl_->node->create_subscription<qyh_standard_robot_msgs::msg::StandardRobotStatus>(
                impl_->cfg.standard_robot_status_topic, rclcpp::SensorDataQoS(),
                [this](qyh_standard_robot_msgs::msg::StandardRobotStatus::ConstSharedPtr msg) {
                    if (!impl_->on_state) return;

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

                    if (impl_->has_nav) {
                        const auto& n = impl_->last_nav;
                        qyh::dataplane::Pose* goal = nav->mutable_current_goal();
                        goal->mutable_position()->set_x(n.autonomous_nav_pose.x);
                        goal->mutable_position()->set_y(n.autonomous_nav_pose.y);
                        goal->mutable_position()->set_z(0.0);
                    }

                    impl_->on_state(out);
                });
        }

        if (!impl_->cfg.arm_state_topic.empty()) {
            impl_->arm_state_sub = impl_->node->create_subscription<qyh_jaka_control_msgs::msg::RobotState>(
                impl_->cfg.arm_state_topic, rclcpp::SensorDataQoS(),
                [this](qyh_jaka_control_msgs::msg::RobotState::ConstSharedPtr msg) {
                    if (!impl_->on_state) return;
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
                    impl_->on_state(out);
                });
        }

        if (!impl_->cfg.lift_state_topic.empty()) {
            impl_->lift_state_sub = impl_->node->create_subscription<qyh_lift_msgs::msg::LiftState>(
                impl_->cfg.lift_state_topic, rclcpp::SensorDataQoS(),
                [this](qyh_lift_msgs::msg::LiftState::ConstSharedPtr msg) {
                    if (!impl_->on_state) return;
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
                    impl_->on_state(out);
                });
        }

        if (!impl_->cfg.waist_state_topic.empty()) {
            impl_->waist_state_sub = impl_->node->create_subscription<qyh_waist_msgs::msg::WaistState>(
                impl_->cfg.waist_state_topic, rclcpp::SensorDataQoS(),
                [this](qyh_waist_msgs::msg::WaistState::ConstSharedPtr msg) {
                    if (!impl_->on_state) return;
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
                    impl_->on_state(out);
                });
        }

        if (!impl_->cfg.head_joint_state_topic.empty()) {
            impl_->head_state_sub = impl_->node->create_subscription<sensor_msgs::msg::JointState>(
                impl_->cfg.head_joint_state_topic, rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
                    if (!impl_->on_state) return;
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
                        impl_->on_state(out);
                    }
                });
        }

        if (!impl_->cfg.left_gripper_state_topic.empty()) {
            impl_->left_gripper_sub = impl_->node->create_subscription<qyh_gripper_msgs::msg::GripperState>(
                impl_->cfg.left_gripper_state_topic, rclcpp::SensorDataQoS(),
                [this](qyh_gripper_msgs::msg::GripperState::ConstSharedPtr msg) {
                    if (!impl_->on_state) return;
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
                    impl_->on_state(out);
                });
        }

        if (!impl_->cfg.right_gripper_state_topic.empty()) {
            impl_->right_gripper_sub = impl_->node->create_subscription<qyh_gripper_msgs::msg::GripperState>(
                impl_->cfg.right_gripper_state_topic, rclcpp::SensorDataQoS(),
                [this](qyh_gripper_msgs::msg::GripperState::ConstSharedPtr msg) {
                    if (!impl_->on_state) return;
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
                    impl_->on_state(out);
                });
        }

        impl_->executor.add_node(impl_->node);
        impl_->spin_thread = std::make_unique<std::thread>([this]() { impl_->executor.spin(); });
        std::cout << "[Ros2ControlStateBridge] started" << std::endl;
        return;
    }
#endif
    std::cout << "[Ros2ControlStateBridge] ROS2 disabled (build without ROBOT_ENDPOINT_ENABLE_ROS2)" << std::endl;
}

void Ros2ControlStateBridge::stop() {
#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    if (impl_->node) {
        impl_->executor.cancel();
        if (impl_->spin_thread && impl_->spin_thread->joinable()) {
            impl_->spin_thread->join();
        }
        impl_->spin_thread.reset();
        impl_->node.reset();
        std::cout << "[Ros2ControlStateBridge] stopped" << std::endl;
    }
#endif
}

} // namespace qyh::robot