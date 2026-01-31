#include "ros2_control_state_impl.hpp"

#include <iostream>
#include <utility>

namespace qyh::robot {

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

void Ros2ControlStateBridge::start() {
#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    if (rclcpp::ok()) {
        impl_->node = std::make_shared<rclcpp::Node>("qyh_robot_endpoint_ctrl");

        detail::setup_control_io(*impl_);
        detail::setup_state_subscriptions(*impl_);

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

} // namespace qyh::robot#include "robot_endpoint/ros2_control_state.hpp"

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
#include <qyh_lift_msgs/srv/lift_control.hpp>
#include <qyh_waist_msgs/srv/waist_control.hpp>
#include <qyh_gripper_msgs/srv/move_gripper.hpp>
#include <qyh_gripper_msgs/srv/activate_gripper.hpp>
#include <qyh_jaka_control_msgs/srv/move_j.hpp>
#include <qyh_jaka_control_msgs/srv/move_l.hpp>
#include <qyh_jaka_control_msgs/srv/jog.hpp>
#include <qyh_jaka_control_msgs/srv/jog_stop.hpp>
#include <qyh_standard_robot_msgs/msg/manual_velocity_command.hpp>
#include <qyh_standard_robot_msgs/srv/go_navigate_to_coordinate.hpp>
#include <qyh_standard_robot_msgs/srv/go_execute_action_task.hpp>
#include <qyh_standard_robot_msgs/srv/go_navigate_to_site_with_task.hpp>
#include <qyh_standard_robot_msgs/srv/control_pause_move.hpp>
#include <qyh_standard_robot_msgs/srv/control_resume_move.hpp>
#include <qyh_standard_robot_msgs/srv/control_stop_move.hpp>
#include <cmath>
#include <array>
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

inline double yaw_from_quat(const qyh::dataplane::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny_cosp, cosy_cosp);
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
    rclcpp::Publisher<qyh_standard_robot_msgs::msg::ManualVelocityCommand>::SharedPtr manual_vel_pub;
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
            geometry_msgs::msg::Twist twist;
            const auto& vel = msg.chassis_vel();
            twist.linear.x = vel.linear_x();
            twist.linear.y = vel.linear_y();
            twist.angular.z = vel.angular_z();
            if (impl_->cmd_vel_pub) {
                impl_->cmd_vel_pub->publish(twist);
            }

            if (impl_->manual_vel_pub) {
                qyh_standard_robot_msgs::msg::ManualVelocityCommand mv;
                mv.vx = vel.linear_x();
                mv.w = vel.angular_z();
                impl_->manual_vel_pub->publish(mv);
            }
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
            uint8_t pos = static_cast<uint8_t>(std::max(0.0, std::min(1.0, gripper.position())) * 255.0);
            uint8_t force = static_cast<uint8_t>(std::max(0.0, std::min(255.0, gripper.force())));
            auto req = std::make_shared<qyh_gripper_msgs::srv::MoveGripper::Request>();
            req->position = pos;
            req->speed = 255;
            req->force = force;
            if (gripper.gripper_id() == "left" && impl_->left_gripper_move_client) {
                impl_->left_gripper_move_client->async_send_request(req);
            } else if (gripper.gripper_id() == "right" && impl_->right_gripper_move_client) {
                impl_->right_gripper_move_client->async_send_request(req);
            } else if (gripper.gripper_id() == "left" && impl_->gripper_left_pub) {
                std_msgs::msg::Float64MultiArray arr;
                arr.data = {gripper.position(), gripper.force()};
                impl_->gripper_left_pub->publish(arr);
            } else if (gripper.gripper_id() == "right" && impl_->gripper_right_pub) {
                std_msgs::msg::Float64MultiArray arr;
                arr.data = {gripper.position(), gripper.force()};
                impl_->gripper_right_pub->publish(arr);
            } else {
                std::cout << "[Ros2ControlStateBridge] unknown gripper_id=" << gripper.gripper_id() << std::endl;
            }
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kHeadCmd: {
            if (!impl_->head_cmd_pub) break;
            const auto& cmd = msg.head_cmd();
            if (cmd.command() != "goto" && cmd.command() != "track") {
                std::cout << "[Ros2ControlStateBridge] head command ignored: " << cmd.command() << std::endl;
                break;
            }
            std_msgs::msg::Float64MultiArray arr;
            double pan = cmd.yaw();
            double tilt = cmd.pitch();
            if (cmd.speed() > 0.0) {
                double speed_pct = std::max(0.0, std::min(1.0, cmd.speed())) * 100.0;
                double duration_ms = 100.0 + (100.0 - speed_pct) * 19.0;
                arr.data = {tilt, pan, duration_ms};
            } else {
                arr.data = {tilt, pan};
            }
            impl_->head_cmd_pub->publish(arr);
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kLiftCmd: {
            if (!impl_->lift_client) break;
            const auto& cmd = msg.lift_cmd();
            auto req = std::make_shared<qyh_lift_msgs::srv::LiftControl::Request>();
            if (cmd.command() == "goto") {
                req->command = qyh_lift_msgs::srv::LiftControl::Request::CMD_GO_POSITION;
                req->value = static_cast<float>(cmd.target_height());
                req->hold = false;
            } else if (cmd.command() == "up") {
                req->command = qyh_lift_msgs::srv::LiftControl::Request::CMD_MOVE_UP;
                req->value = static_cast<float>(cmd.speed());
                req->hold = true;
            } else if (cmd.command() == "down") {
                req->command = qyh_lift_msgs::srv::LiftControl::Request::CMD_MOVE_DOWN;
                req->value = static_cast<float>(cmd.speed());
                req->hold = true;
            } else if (cmd.command() == "stop") {
                req->command = qyh_lift_msgs::srv::LiftControl::Request::CMD_STOP;
                req->value = 0.0f;
                req->hold = false;
            } else {
                std::cout << "[Ros2ControlStateBridge] lift command ignored: " << cmd.command() << std::endl;
                break;
            }
            impl_->lift_client->async_send_request(req);
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kWaistCmd: {
            if (!impl_->waist_client) break;
            const auto& cmd = msg.waist_cmd();
            auto req = std::make_shared<qyh_waist_msgs::srv::WaistControl::Request>();
            if (cmd.command() == "goto") {
                req->command = qyh_waist_msgs::srv::WaistControl::Request::CMD_GO_ANGLE;
                req->value = static_cast<float>(cmd.target_angle() * 180.0 / M_PI);
                req->hold = false;
            } else if (cmd.command() == "left") {
                req->command = qyh_waist_msgs::srv::WaistControl::Request::CMD_LEAN_FORWARD;
                req->value = static_cast<float>(cmd.speed());
                req->hold = true;
            } else if (cmd.command() == "right") {
                req->command = qyh_waist_msgs::srv::WaistControl::Request::CMD_LEAN_BACK;
                req->value = static_cast<float>(cmd.speed());
                req->hold = true;
            } else if (cmd.command() == "stop") {
                req->command = qyh_waist_msgs::srv::WaistControl::Request::CMD_STOP;
                req->value = 0.0f;
                req->hold = false;
            } else {
                std::cout << "[Ros2ControlStateBridge] waist command ignored: " << cmd.command() << std::endl;
                break;
            }
            impl_->waist_client->async_send_request(req);
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kNavGoal: {
            const auto& cmd = msg.nav_goal();
            const auto& frame = cmd.header().frame_id();
            if (!frame.empty() && frame.rfind("site:", 0) == 0) {
                if (!impl_->nav_site_client) break;
                uint16_t site_id = static_cast<uint16_t>(std::stoi(frame.substr(5)));
                auto req = std::make_shared<qyh_standard_robot_msgs::srv::GoExecuteActionTask::Request>();
                req->site_id = site_id;
                req->is_localization = cmd.is_localization_only();
                impl_->nav_site_client->async_send_request(req);
                break;
            }
            if (!frame.empty() && frame.rfind("site_task:", 0) == 0) {
                if (!impl_->nav_site_task_client) break;
                auto payload = frame.substr(10);
                auto sep = payload.find(':');
                if (sep == std::string::npos) break;
                uint16_t site_id = static_cast<uint16_t>(std::stoi(payload.substr(0, sep)));
                uint16_t task_id = static_cast<uint16_t>(std::stoi(payload.substr(sep + 1)));
                auto req = std::make_shared<qyh_standard_robot_msgs::srv::GoNavigateToSiteWithTask::Request>();
                req->site_id = site_id;
                req->task_id = task_id;
                impl_->nav_site_task_client->async_send_request(req);
                break;
            }
            if (!impl_->nav_coord_client) break;
            auto req = std::make_shared<qyh_standard_robot_msgs::srv::GoNavigateToCoordinate::Request>();
            req->x = cmd.target_pose().position().x();
            req->y = cmd.target_pose().position().y();
            req->yaw = yaw_from_quat(cmd.target_pose().orientation());
            req->is_localization = cmd.is_localization_only();
            impl_->nav_coord_client->async_send_request(req);
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kNavCtrl: {
            const auto& cmd = msg.nav_ctrl();
            if (cmd.action() == "pause" && impl_->nav_pause_client) {
                impl_->nav_pause_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlPauseMove::Request>());
            } else if (cmd.action() == "resume" && impl_->nav_resume_client) {
                impl_->nav_resume_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlResumeMove::Request>());
            } else if (cmd.action() == "cancel" && impl_->nav_cancel_client) {
                impl_->nav_cancel_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlStopMove::Request>());
            } else {
                std::cout << "[Ros2ControlStateBridge] nav control ignored: " << cmd.action() << std::endl;
            }
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kArmMove: {
            const auto& cmd = msg.arm_move();
            int robot_id = 0;
            if (cmd.arm_side() == "right") robot_id = 1;
            if (cmd.arm_side() == "dual") robot_id = -1;
            if (cmd.motion_type() == "movej" && impl_->arm_movej_client) {
                auto req = std::make_shared<qyh_jaka_control_msgs::srv::MoveJ::Request>();
                req->robot_id = robot_id;
                std::array<double, 14> joints{};
                joints.fill(0.0);
                if (cmd.target_size() == 14) {
                    for (int i = 0; i < 14; ++i) {
                        joints[i] = cmd.target(i);
                    }
                } else if (cmd.target_size() == 7) {
                    int offset = (robot_id == 1) ? 7 : 0;
                    for (int i = 0; i < 7; ++i) {
                        joints[offset + i] = cmd.target(i);
                    }
                }
                for (int i = 0; i < 14; ++i) {
                    req->joint_positions[i] = joints[i];
                }
                req->move_mode = false;
                req->velocity = cmd.speed();
                req->acceleration = cmd.acceleration();
                req->is_block = false;
                impl_->arm_movej_client->async_send_request(req);
            } else if (cmd.motion_type() == "movel" && impl_->arm_movel_client) {
                if (cmd.target_size() < 6) break;
                auto req = std::make_shared<qyh_jaka_control_msgs::srv::MoveL::Request>();
                req->robot_id = robot_id;
                req->target_pose.position.x = cmd.target(0);
                req->target_pose.position.y = cmd.target(1);
                req->target_pose.position.z = cmd.target(2);
                req->target_pose.orientation.x = cmd.target(3);
                req->target_pose.orientation.y = cmd.target(4);
                req->target_pose.orientation.z = cmd.target(5);
                req->target_pose.orientation.w = 1.0;
                req->move_mode = false;
                req->velocity = cmd.speed();
                req->acceleration = cmd.acceleration();
                req->is_block = false;
                impl_->arm_movel_client->async_send_request(req);
            }
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kArmJog: {
            if (!impl_->arm_jog_client) break;
            const auto& cmd = msg.arm_jog();
            int robot_id = cmd.arm_side() == "right" ? 1 : 0;
            auto req = std::make_shared<qyh_jaka_control_msgs::srv::Jog::Request>();
            req->robot_id = robot_id;
            req->axis_num = static_cast<int8_t>(cmd.axis_index() + 1);
            req->move_mode = qyh_jaka_control_msgs::srv::Jog::Request::MOVE_CONTINUOUS;
            req->coord_type = (cmd.jog_mode() == "cartesian")
                                  ? qyh_jaka_control_msgs::srv::Jog::Request::COORD_TOOL
                                  : qyh_jaka_control_msgs::srv::Jog::Request::COORD_JOINT;
            req->velocity = cmd.direction();
            req->position = 0.0;
            if (std::abs(cmd.direction()) < 1e-6 && impl_->arm_jog_stop_client) {
                auto stop = std::make_shared<qyh_jaka_control_msgs::srv::JogStop::Request>();
                stop->robot_id = robot_id;
                stop->axis_num = req->axis_num;
                impl_->arm_jog_stop_client->async_send_request(stop);
            } else {
                impl_->arm_jog_client->async_send_request(req);
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
        if (!impl_->cfg.manual_velocity_topic.empty()) {
            impl_->manual_vel_pub = impl_->node->create_publisher<qyh_standard_robot_msgs::msg::ManualVelocityCommand>(
                impl_->cfg.manual_velocity_topic, 10);
        }
        impl_->joint_cmd_pub = impl_->node->create_publisher<sensor_msgs::msg::JointState>(
            impl_->cfg.joint_command_topic, 10);

        if (!impl_->cfg.head_command_topic.empty()) {
            impl_->head_cmd_pub = impl_->node->create_publisher<std_msgs::msg::Float64MultiArray>(
                impl_->cfg.head_command_topic, 10);
        }

        impl_->gripper_left_pub = impl_->node->create_publisher<std_msgs::msg::Float64MultiArray>(
            impl_->cfg.gripper_command_topic_prefix + "/left", 10);
        impl_->gripper_right_pub = impl_->node->create_publisher<std_msgs::msg::Float64MultiArray>(
            impl_->cfg.gripper_command_topic_prefix + "/right", 10);

        if (!impl_->cfg.lift_control_service.empty()) {
            impl_->lift_client = impl_->node->create_client<qyh_lift_msgs::srv::LiftControl>(
                impl_->cfg.lift_control_service);
        }
        if (!impl_->cfg.waist_control_service.empty()) {
            impl_->waist_client = impl_->node->create_client<qyh_waist_msgs::srv::WaistControl>(
                impl_->cfg.waist_control_service);
        }
        if (!impl_->cfg.left_gripper_activate_service.empty()) {
            impl_->left_gripper_activate_client = impl_->node->create_client<qyh_gripper_msgs::srv::ActivateGripper>(
                impl_->cfg.left_gripper_activate_service);
        }
        if (!impl_->cfg.right_gripper_activate_service.empty()) {
            impl_->right_gripper_activate_client = impl_->node->create_client<qyh_gripper_msgs::srv::ActivateGripper>(
                impl_->cfg.right_gripper_activate_service);
        }
        if (!impl_->cfg.left_gripper_move_service.empty()) {
            impl_->left_gripper_move_client = impl_->node->create_client<qyh_gripper_msgs::srv::MoveGripper>(
                impl_->cfg.left_gripper_move_service);
        }
        if (!impl_->cfg.right_gripper_move_service.empty()) {
            impl_->right_gripper_move_client = impl_->node->create_client<qyh_gripper_msgs::srv::MoveGripper>(
                impl_->cfg.right_gripper_move_service);
        }
        if (!impl_->cfg.arm_movej_service.empty()) {
            impl_->arm_movej_client = impl_->node->create_client<qyh_jaka_control_msgs::srv::MoveJ>(
                impl_->cfg.arm_movej_service);
        }
        if (!impl_->cfg.arm_movel_service.empty()) {
            impl_->arm_movel_client = impl_->node->create_client<qyh_jaka_control_msgs::srv::MoveL>(
                impl_->cfg.arm_movel_service);
        }
        if (!impl_->cfg.arm_jog_service.empty()) {
            impl_->arm_jog_client = impl_->node->create_client<qyh_jaka_control_msgs::srv::Jog>(
                impl_->cfg.arm_jog_service);
        }
        if (!impl_->cfg.arm_jog_stop_service.empty()) {
            impl_->arm_jog_stop_client = impl_->node->create_client<qyh_jaka_control_msgs::srv::JogStop>(
                impl_->cfg.arm_jog_stop_service);
        }
        if (!impl_->cfg.nav_to_coordinate_service.empty()) {
            impl_->nav_coord_client = impl_->node->create_client<qyh_standard_robot_msgs::srv::GoNavigateToCoordinate>(
                impl_->cfg.nav_to_coordinate_service);
        }
        if (!impl_->cfg.nav_to_site_service.empty()) {
            impl_->nav_site_client = impl_->node->create_client<qyh_standard_robot_msgs::srv::GoExecuteActionTask>(
                impl_->cfg.nav_to_site_service);
        }
        if (!impl_->cfg.nav_to_site_task_service.empty()) {
            impl_->nav_site_task_client = impl_->node->create_client<qyh_standard_robot_msgs::srv::GoNavigateToSiteWithTask>(
                impl_->cfg.nav_to_site_task_service);
        }
        if (!impl_->cfg.nav_pause_service.empty()) {
            impl_->nav_pause_client = impl_->node->create_client<qyh_standard_robot_msgs::srv::ControlPauseMove>(
                impl_->cfg.nav_pause_service);
        }
        if (!impl_->cfg.nav_resume_service.empty()) {
            impl_->nav_resume_client = impl_->node->create_client<qyh_standard_robot_msgs::srv::ControlResumeMove>(
                impl_->cfg.nav_resume_service);
        }
        if (!impl_->cfg.nav_cancel_service.empty()) {
            impl_->nav_cancel_client = impl_->node->create_client<qyh_standard_robot_msgs::srv::ControlStopMove>(
                impl_->cfg.nav_cancel_service);
        }

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