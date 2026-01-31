#include "ros2_control_state_impl.hpp"

#include <iostream>

namespace qyh::robot {

namespace detail {
void setup_control_io(Ros2ControlStateBridge::Impl& impl) {
#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    impl.cmd_vel_pub = impl.node->create_publisher<geometry_msgs::msg::Twist>(
        impl.cfg.cmd_vel_topic, 10);
    if (!impl.cfg.manual_velocity_topic.empty()) {
        impl.manual_vel_pub = impl.node->create_publisher<qyh_standard_robot_msgs::msg::ManualVelocityCommand>(
            impl.cfg.manual_velocity_topic, 10);
    }
    if (!impl.cfg.manual_motion_topic.empty()) {
        impl.manual_motion_pub = impl.node->create_publisher<qyh_standard_robot_msgs::msg::ManualMotionCommand>(
            impl.cfg.manual_motion_topic, 10);
    }
    impl.joint_cmd_pub = impl.node->create_publisher<sensor_msgs::msg::JointState>(
        impl.cfg.joint_command_topic, 10);

    if (!impl.cfg.head_command_topic.empty()) {
        impl.head_cmd_pub = impl.node->create_publisher<std_msgs::msg::Float64MultiArray>(
            impl.cfg.head_command_topic, 10);
    }

    impl.gripper_left_pub = impl.node->create_publisher<std_msgs::msg::Float64MultiArray>(
        impl.cfg.gripper_command_topic_prefix + "/left", 10);
    impl.gripper_right_pub = impl.node->create_publisher<std_msgs::msg::Float64MultiArray>(
        impl.cfg.gripper_command_topic_prefix + "/right", 10);

    if (!impl.cfg.lift_control_service.empty()) {
        impl.lift_client = impl.node->create_client<qyh_lift_msgs::srv::LiftControl>(
            impl.cfg.lift_control_service);
    }
    if (!impl.cfg.waist_control_service.empty()) {
        impl.waist_client = impl.node->create_client<qyh_waist_msgs::srv::WaistControl>(
            impl.cfg.waist_control_service);
    }
    if (!impl.cfg.left_gripper_activate_service.empty()) {
        impl.left_gripper_activate_client = impl.node->create_client<qyh_gripper_msgs::srv::ActivateGripper>(
            impl.cfg.left_gripper_activate_service);
    }
    if (!impl.cfg.right_gripper_activate_service.empty()) {
        impl.right_gripper_activate_client = impl.node->create_client<qyh_gripper_msgs::srv::ActivateGripper>(
            impl.cfg.right_gripper_activate_service);
    }
    if (!impl.cfg.left_gripper_move_service.empty()) {
        impl.left_gripper_move_client = impl.node->create_client<qyh_gripper_msgs::srv::MoveGripper>(
            impl.cfg.left_gripper_move_service);
    }
    if (!impl.cfg.right_gripper_move_service.empty()) {
        impl.right_gripper_move_client = impl.node->create_client<qyh_gripper_msgs::srv::MoveGripper>(
            impl.cfg.right_gripper_move_service);
    }
    if (!impl.cfg.arm_movej_service.empty()) {
        impl.arm_movej_client = impl.node->create_client<qyh_jaka_control_msgs::srv::MoveJ>(
            impl.cfg.arm_movej_service);
    }
    if (!impl.cfg.arm_movel_service.empty()) {
        impl.arm_movel_client = impl.node->create_client<qyh_jaka_control_msgs::srv::MoveL>(
            impl.cfg.arm_movel_service);
    }
    if (!impl.cfg.arm_jog_service.empty()) {
        impl.arm_jog_client = impl.node->create_client<qyh_jaka_control_msgs::srv::Jog>(
            impl.cfg.arm_jog_service);
    }
    if (!impl.cfg.arm_jog_stop_service.empty()) {
        impl.arm_jog_stop_client = impl.node->create_client<qyh_jaka_control_msgs::srv::JogStop>(
            impl.cfg.arm_jog_stop_service);
    }
    if (!impl.cfg.nav_to_coordinate_service.empty()) {
        impl.nav_coord_client = impl.node->create_client<qyh_standard_robot_msgs::srv::GoNavigateToCoordinate>(
            impl.cfg.nav_to_coordinate_service);
    }
    if (!impl.cfg.nav_to_site_service.empty()) {
        impl.nav_site_client = impl.node->create_client<qyh_standard_robot_msgs::srv::GoExecuteActionTask>(
            impl.cfg.nav_to_site_service);
    }
    if (!impl.cfg.nav_to_site_task_service.empty()) {
        impl.nav_site_task_client = impl.node->create_client<qyh_standard_robot_msgs::srv::GoNavigateToSiteWithTask>(
            impl.cfg.nav_to_site_task_service);
    }
    if (!impl.cfg.nav_pause_service.empty()) {
        impl.nav_pause_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlPauseMove>(
            impl.cfg.nav_pause_service);
    }
    if (!impl.cfg.nav_resume_service.empty()) {
        impl.nav_resume_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlResumeMove>(
            impl.cfg.nav_resume_service);
    }
    if (!impl.cfg.nav_cancel_service.empty()) {
        impl.nav_cancel_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlStopMove>(
            impl.cfg.nav_cancel_service);
    }

    if (!impl.cfg.chassis_emergency_stop_service.empty()) {
        impl.chassis_emergency_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlEmergencyStop>(
            impl.cfg.chassis_emergency_stop_service);
    }
    if (!impl.cfg.chassis_release_emergency_service.empty()) {
        impl.chassis_release_emergency_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlReleaseEmergencyStop>(
            impl.cfg.chassis_release_emergency_service);
    }
    if (!impl.cfg.chassis_start_charging_service.empty()) {
        impl.chassis_start_charging_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlStartCharging>(
            impl.cfg.chassis_start_charging_service);
    }
    if (!impl.cfg.chassis_stop_charging_service.empty()) {
        impl.chassis_stop_charging_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlStopCharging>(
            impl.cfg.chassis_stop_charging_service);
    }
    if (!impl.cfg.chassis_enter_low_power_service.empty()) {
        impl.chassis_enter_low_power_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlEnterLowPowerMode>(
            impl.cfg.chassis_enter_low_power_service);
    }
    if (!impl.cfg.chassis_exit_low_power_service.empty()) {
        impl.chassis_exit_low_power_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlExitLowPowerMode>(
            impl.cfg.chassis_exit_low_power_service);
    }
    if (!impl.cfg.chassis_start_manual_service.empty()) {
        impl.chassis_start_manual_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlStartManualControl>(
            impl.cfg.chassis_start_manual_service);
    }
    if (!impl.cfg.chassis_stop_manual_service.empty()) {
        impl.chassis_stop_manual_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlStopManualControl>(
            impl.cfg.chassis_stop_manual_service);
    }
    if (!impl.cfg.chassis_pause_mission_service.empty()) {
        impl.chassis_pause_mission_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlPauseMission>(
            impl.cfg.chassis_pause_mission_service);
    }
    if (!impl.cfg.chassis_resume_mission_service.empty()) {
        impl.chassis_resume_mission_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlResumeMission>(
            impl.cfg.chassis_resume_mission_service);
    }
    if (!impl.cfg.chassis_cancel_mission_service.empty()) {
        impl.chassis_cancel_mission_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlCancelMission>(
            impl.cfg.chassis_cancel_mission_service);
    }
    if (!impl.cfg.chassis_stop_localization_service.empty()) {
        impl.chassis_stop_localization_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlStopLocalization>(
            impl.cfg.chassis_stop_localization_service);
    }
    if (!impl.cfg.chassis_system_reset_service.empty()) {
        impl.chassis_system_reset_client = impl.node->create_client<qyh_standard_robot_msgs::srv::ControlSystemReset>(
            impl.cfg.chassis_system_reset_service);
    }

    if (!impl.cfg.task_execute_service.empty()) {
        impl.task_execute_client = impl.node->create_client<qyh_task_engine_msgs::srv::ExecuteTask>(
            impl.cfg.task_execute_service);
    }
    if (!impl.cfg.task_pause_service.empty()) {
        impl.task_pause_client = impl.node->create_client<qyh_task_engine_msgs::srv::PauseTask>(
            impl.cfg.task_pause_service);
    }
    if (!impl.cfg.task_resume_service.empty()) {
        impl.task_resume_client = impl.node->create_client<qyh_task_engine_msgs::srv::ResumeTask>(
            impl.cfg.task_resume_service);
    }
    if (!impl.cfg.task_cancel_service.empty()) {
        impl.task_cancel_client = impl.node->create_client<qyh_task_engine_msgs::srv::CancelTask>(
            impl.cfg.task_cancel_service);
    }

    if (!impl.cfg.led_color_topic.empty()) {
        impl.led_color_pub = impl.node->create_publisher<std_msgs::msg::ColorRGBA>(
            impl.cfg.led_color_topic, 10);
    }
    if (!impl.cfg.led_blink_topic.empty()) {
        impl.led_blink_pub = impl.node->create_publisher<std_msgs::msg::String>(
            impl.cfg.led_blink_topic, 10);
    }

    if (!impl.cfg.shutdown_service.empty()) {
        impl.shutdown_client = impl.node->create_client<std_srvs::srv::Trigger>(
            impl.cfg.shutdown_service);
    }
#else
    (void)impl;
#endif
}
} // namespace detail

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
        case qyh::dataplane::ControlChannelMessage::kChassisManual: {
            if (!impl_->manual_motion_pub) break;
            const auto& cmd = msg.chassis_manual();
            qyh_standard_robot_msgs::msg::ManualMotionCommand mm;
            mm.forward = cmd.forward();
            mm.backward = cmd.backward();
            mm.left = cmd.left();
            mm.right = cmd.right();
            mm.rotate_left = cmd.rotate_left();
            mm.rotate_right = cmd.rotate_right();
            impl_->manual_motion_pub->publish(mm);
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kChassisControl: {
            const auto& cmd = msg.chassis_control();
            const auto& action = cmd.action();
            if (action == "emergency_stop" && impl_->chassis_emergency_client) {
                impl_->chassis_emergency_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlEmergencyStop::Request>());
            } else if (action == "release_emergency_stop" && impl_->chassis_release_emergency_client) {
                impl_->chassis_release_emergency_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlReleaseEmergencyStop::Request>());
            } else if (action == "start_charging" && impl_->chassis_start_charging_client) {
                impl_->chassis_start_charging_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlStartCharging::Request>());
            } else if (action == "stop_charging" && impl_->chassis_stop_charging_client) {
                impl_->chassis_stop_charging_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlStopCharging::Request>());
            } else if (action == "enter_low_power" && impl_->chassis_enter_low_power_client) {
                impl_->chassis_enter_low_power_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlEnterLowPowerMode::Request>());
            } else if (action == "exit_low_power" && impl_->chassis_exit_low_power_client) {
                impl_->chassis_exit_low_power_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlExitLowPowerMode::Request>());
            } else if (action == "start_manual" && impl_->chassis_start_manual_client) {
                impl_->chassis_start_manual_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlStartManualControl::Request>());
            } else if (action == "stop_manual" && impl_->chassis_stop_manual_client) {
                impl_->chassis_stop_manual_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlStopManualControl::Request>());
            } else if (action == "pause_mission" && impl_->chassis_pause_mission_client) {
                impl_->chassis_pause_mission_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlPauseMission::Request>());
            } else if (action == "resume_mission" && impl_->chassis_resume_mission_client) {
                impl_->chassis_resume_mission_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlResumeMission::Request>());
            } else if (action == "cancel_mission" && impl_->chassis_cancel_mission_client) {
                impl_->chassis_cancel_mission_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlCancelMission::Request>());
            } else if (action == "stop_localization" && impl_->chassis_stop_localization_client) {
                impl_->chassis_stop_localization_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlStopLocalization::Request>());
            } else if (action == "system_reset" && impl_->chassis_system_reset_client) {
                impl_->chassis_system_reset_client->async_send_request(
                    std::make_shared<qyh_standard_robot_msgs::srv::ControlSystemReset::Request>());
            } else {
                std::cout << "[Ros2ControlStateBridge] chassis control ignored: " << action << std::endl;
            }
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kTaskControl: {
            const auto& cmd = msg.task_control();
            const auto& action = cmd.action();
            if (action == "execute" && impl_->task_execute_client) {
                auto req = std::make_shared<qyh_task_engine_msgs::srv::ExecuteTask::Request>();
                req->task_json = cmd.task_json();
                impl_->task_execute_client->async_send_request(req);
            } else if (action == "pause" && impl_->task_pause_client) {
                auto req = std::make_shared<qyh_task_engine_msgs::srv::PauseTask::Request>();
                req->task_id = cmd.task_id();
                impl_->task_pause_client->async_send_request(req);
            } else if (action == "resume" && impl_->task_resume_client) {
                auto req = std::make_shared<qyh_task_engine_msgs::srv::ResumeTask::Request>();
                req->task_id = cmd.task_id();
                impl_->task_resume_client->async_send_request(req);
            } else if (action == "cancel" && impl_->task_cancel_client) {
                auto req = std::make_shared<qyh_task_engine_msgs::srv::CancelTask::Request>();
                req->task_id = cmd.task_id();
                impl_->task_cancel_client->async_send_request(req);
            } else {
                std::cout << "[Ros2ControlStateBridge] task control ignored: " << action << std::endl;
            }
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kLedCommand: {
            const auto& cmd = msg.led_command();
            if (cmd.mode() == "color" && impl_->led_color_pub) {
                std_msgs::msg::ColorRGBA c;
                c.r = std::min(255u, cmd.r()) / 255.0f;
                c.g = std::min(255u, cmd.g()) / 255.0f;
                c.b = std::min(255u, cmd.b()) / 255.0f;
                c.a = std::min(255u, cmd.w()) / 255.0f;
                impl_->led_color_pub->publish(c);
            } else if (cmd.mode() == "blink" && impl_->led_blink_pub) {
                std_msgs::msg::String s;
                s.data = cmd.pattern();
                impl_->led_blink_pub->publish(s);
            } else if (cmd.mode() == "stop" && impl_->led_blink_pub) {
                std_msgs::msg::String s;
                s.data = "stop";
                impl_->led_blink_pub->publish(s);
            } else {
                std::cout << "[Ros2ControlStateBridge] led command ignored: " << cmd.mode() << std::endl;
            }
            break;
        }
        case qyh::dataplane::ControlChannelMessage::kShutdown: {
            const auto& cmd = msg.shutdown();
            if (cmd.trigger() && impl_->shutdown_client) {
                impl_->shutdown_client->async_send_request(
                    std::make_shared<std_srvs::srv::Trigger::Request>());
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
        case qyh::dataplane::ControlChannelMessage::kGripperActivate: {
            const auto& gripper = msg.gripper_activate();
            if (!gripper.activate()) {
                std::cout << "[Ros2ControlStateBridge] gripper deactivate ignored" << std::endl;
                break;
            }
            if (gripper.gripper_id() == "left" && impl_->left_gripper_activate_client) {
                impl_->left_gripper_activate_client->async_send_request(
                    std::make_shared<qyh_gripper_msgs::srv::ActivateGripper::Request>());
            } else if (gripper.gripper_id() == "right" && impl_->right_gripper_activate_client) {
                impl_->right_gripper_activate_client->async_send_request(
                    std::make_shared<qyh_gripper_msgs::srv::ActivateGripper::Request>());
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

} // namespace qyh::robot