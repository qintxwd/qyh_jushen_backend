/**
 * @file message_handler.cpp
 * @brief 消息处理器实现
 * 
 * 负责解析和处理所有入站 WebSocket 消息，
 * 包括认证、订阅、心跳和各类控制命令
 */

#include "data_plane/message_handler.hpp"
#include "data_plane/session.hpp"
#include "data_plane/auth.hpp"
#include "data_plane/state_cache.hpp"
#include "data_plane/config.hpp"
#include "data_plane/control_sync.hpp"
#include "data_plane/vr_session.hpp"
#include "data_plane/server.hpp"
#include "data_plane/connection_manager.hpp"

#include "data_plane/ros2_bridge.hpp"
#include "data_plane/watchdog.hpp"

// Protobuf 生成的头文件
#include "messages.pb.h"
#include "control.pb.h"
#include "state.pb.h"
#include "common.pb.h"

#include <iostream>
#include <chrono>

namespace qyh::dataplane {

MessageHandler::MessageHandler(const Config& config,
                               JWTValidator& validator,
                               StateCache& state_cache)
    : config_(config)
    , validator_(validator)
    , state_cache_(state_cache)
{
}

void MessageHandler::handle_message(std::shared_ptr<Session> session,
                                     const std::vector<uint8_t>& data) {
    std::cout << "[MessageHandler] ===== 收到消息 =====" << std::endl;
    std::cout << "[MessageHandler] 数据大小: " << data.size() << " 字节" << std::endl;
    if (data.size() > 0) {
        std::cout << "[MessageHandler] 前10字节: ";
        for (size_t i = 0; i < std::min(data.size(), (size_t)10); i++) {
            std::cout << std::hex << (int)data[i] << " ";
        }
        std::cout << std::dec << std::endl;
    }
    // 解析 WebSocketMessage
    qyh::dataplane::WebSocketMessage msg;
    if (!msg.ParseFromArray(data.data(), static_cast<int>(data.size()))) {
        std::cerr << "[MessageHandler] Failed to parse WebSocketMessage, size=" 
                  << data.size() << std::endl;
        send_error(session, 400, "Invalid message format");
        return;
    }
    std::cout << "[MessageHandler] ✓ Protobuf解析成功, type=" << msg.type() << std::endl;
    std::cout << "[MessageHandler] ✓ Protobuf解析成功, type=" << msg.type() << std::endl;
    
    // 检查鉴权状态
    if (session->state() == SessionState::CONNECTING) {
        // 未认证时只能发送认证请求
        if (msg.type() != qyh::dataplane::MSG_AUTH_REQUEST) {
            send_error(session, 401, "Authentication required");
            return;
        }
    }
    
    // 根据消息类型分发处理
    std::cout << "[MessageHandler] 收到消息类型: " << msg.type() << std::endl;
    switch (msg.type()) {
        case qyh::dataplane::MSG_AUTH_REQUEST:
            handle_auth_request(session, msg);
            break;
            
        case qyh::dataplane::MSG_SUBSCRIBE:
            handle_subscribe(session, msg);
            break;
            
        case qyh::dataplane::MSG_UNSUBSCRIBE:
            handle_unsubscribe(session, msg);
            break;
            
        case qyh::dataplane::MSG_HEARTBEAT:
            handle_heartbeat(session, msg);
            break;
            
        case qyh::dataplane::MSG_VR_CONTROL:
            handle_vr_control(session, msg);
            break;
            
        case qyh::dataplane::MSG_CHASSIS_VELOCITY:
            handle_chassis_velocity(session, msg);
            break;
            
        case qyh::dataplane::MSG_JOINT_COMMAND:
            handle_joint_command(session, msg);
            break;
            
        case qyh::dataplane::MSG_END_EFFECTOR_CMD:
            handle_end_effector_command(session, msg);
            break;
            
        case qyh::dataplane::MSG_GRIPPER_COMMAND:
            handle_gripper_command(session, msg);
            break;
            
        case qyh::dataplane::MSG_NAVIGATION_GOAL:
            handle_navigation_goal(session, msg);
            break;
            
        case qyh::dataplane::MSG_EMERGENCY_STOP:
            handle_emergency_stop(session, msg);
            break;
        
        case qyh::dataplane::MSG_NAVIGATION_CANCEL:
        case qyh::dataplane::MSG_NAVIGATION_PAUSE:
        case qyh::dataplane::MSG_NAVIGATION_RESUME:
            handle_navigation_control(session, msg);
            break;
        
        case qyh::dataplane::MSG_LIFT_COMMAND:
            handle_lift_command(session, msg);
            break;
        
        case qyh::dataplane::MSG_WAIST_COMMAND:
            handle_waist_command(session, msg);
            break;
        
        case qyh::dataplane::MSG_HEAD_COMMAND:
            handle_head_command(session, msg);
            break;
        
        case qyh::dataplane::MSG_ARM_MOVE:
            handle_arm_move(session, msg);
            break;
        
        case qyh::dataplane::MSG_ARM_JOG:
            handle_arm_jog(session, msg);
            break;
            
        default:
            std::cerr << "[MessageHandler] Unknown message type: " 
                      << static_cast<int>(msg.type()) << std::endl;
            send_error(session, 400, "Unknown message type");
            break;
    }
}

void MessageHandler::handle_auth_request(std::shared_ptr<Session> session,
                                          const WebSocketMessage& msg) {
    std::cout << "[MessageHandler] ===== handle_auth_request 开始 =====" << std::endl;
    
    // 从消息中提取认证请求
    if (!msg.has_auth_request()) {
        std::cout << "[MessageHandler] ERROR: msg.has_auth_request() 返回 false!" << std::endl;
        send_error(session, 400, "Missing auth_request payload");
        return;
    }
    
    const auto& auth_req = msg.auth_request();
    
    // 如果禁用认证，直接通过
    if (!config_.auth.enabled) {
        std::cout << "[MessageHandler] Auth disabled, auto-passing session " 
                  << session->session_id() << std::endl;
        session->set_client_type(auth_req.client_type());
        session->set_client_version(auth_req.client_version());
        session->mark_authenticated();
        session->update_connection_info(
            auth_req.client_type() == "vr" ? ConnectionPriority::CRITICAL
                                            : ConnectionPriority::NORMAL
        );
        send_auth_response(session, true);
        return;
    }
    
    // 获取 Token
    std::string token = auth_req.token();
    if (token.empty()) {
        send_auth_response(session, false, "Token is required");
        return;
    }
    
    // 验证 Token
    std::cout << "[MessageHandler] 开始验证Token, 长度: " << token.length() << std::endl;
    auto user_info = validator_.validate(token);
    std::cout << "[MessageHandler] Token验证完成, 结果: " << (user_info ? "有效" : "无效") << std::endl;
    if (!user_info) {
        send_auth_response(session, false, "Invalid token");
        return;
    }
    
    // 检查是否过期
    if (validator_.is_expired(token)) {
        send_auth_response(session, false, "Token expired");
        return;
    }
    
    // 认证成功，记录客户端信息
    session->set_user_info(*user_info);
    session->set_client_type(auth_req.client_type());
    session->set_client_version(auth_req.client_version());
    session->mark_authenticated();

    ConnectionPriority priority = ConnectionPriority::NORMAL;
    if (auth_req.client_type() == "vr") {
        priority = ConnectionPriority::CRITICAL;
    } else if (user_info->role == "admin" || user_info->role == "operator") {
        priority = ConnectionPriority::HIGH;
    }
    session->update_connection_info(priority);

    if (control_sync_) {
        control_sync_->associate_session(session->session_id(), user_info->user_id);
    }
    
    // VR 专用通道: 如果是 VR 客户端，尝试注册到 VRSessionManager
    if (auth_req.client_type() == "vr") {
        bool accepted = VRSessionManager::instance().try_accept(
            session,
            "PICO 4",  // TODO: 从 auth_req 扩展字段获取设备信息
            auth_req.client_version()
        );
        
        if (!accepted) {
            // VR 通道已被占用，拒绝连接
            send_auth_response(session, false, "VR channel already occupied");
            session->close();
            return;
        }
    }
    
    std::cout << "[MessageHandler] Session " << session->session_id() 
              << " authenticated as " << user_info->username 
              << " (" << auth_req.client_type() << " v" << auth_req.client_version() << ")"
              << std::endl;
    
    std::cout << "[MessageHandler] 准备发送认证成功响应..." << std::endl;
    send_auth_response(session, true);
    std::cout << "[MessageHandler] ===== handle_auth_request 结束 (成功) =====" << std::endl;
}

void MessageHandler::handle_subscribe(std::shared_ptr<Session> session,
                                       const WebSocketMessage& msg) {
    if (!msg.has_subscribe()) {
        send_error(session, 400, "Missing subscribe payload");
        return;
    }
    
    const auto& sub_req = msg.subscribe();
    
    // 订阅请求的话题
    for (const auto& topic : sub_req.topics()) {
        session->subscribe(topic);
        std::cout << "[MessageHandler] Session " << session->session_id() 
                  << " subscribed to: " << topic << std::endl;
    }
    
    // 设置最大推送频率
    if (sub_req.max_rate_hz() > 0) {
        session->set_max_push_rate(sub_req.max_rate_hz());
    }
    
    session->set_state(SessionState::ACTIVE);
}

void MessageHandler::handle_unsubscribe(std::shared_ptr<Session> session,
                                         const WebSocketMessage& msg) {
    if (!msg.has_unsubscribe()) {
        send_error(session, 400, "Missing unsubscribe payload");
        return;
    }
    
    const auto& unsub_req = msg.unsubscribe();
    
    for (const auto& topic : unsub_req.topics()) {
        session->unsubscribe(topic);
        std::cout << "[MessageHandler] Session " << session->session_id() 
                  << " unsubscribed from: " << topic << std::endl;
    }
}

void MessageHandler::handle_heartbeat(std::shared_ptr<Session> session,
                                       const WebSocketMessage& msg) {
    session->update_heartbeat();
    
    if (watchdog_) {
        watchdog_->feed(session->session_id());
    }
    
    // 回复心跳
    qyh::dataplane::WebSocketMessage response;
    response.set_type(qyh::dataplane::MSG_HEARTBEAT_ACK);
    response.set_sequence(msg.sequence());
    
    set_timestamp(response.mutable_timestamp());
    
    std::vector<uint8_t> data(response.ByteSizeLong());
    response.SerializeToArray(data.data(), static_cast<int>(data.size()));
    session->send(data);
}

void MessageHandler::handle_vr_control(std::shared_ptr<Session> session,
                                        const WebSocketMessage& msg) {
    // VR 专用通道: 检查是否是 VR 会话
    // VR 会话已通过 VRSessionManager 单连接限制，无需额外控制权检查
    if (!VRSessionManager::instance().is_vr_session(session->session_id())) {
        // 非 VR 会话，回退到传统控制权检查
        if (!session->has_control_permission()) {
            send_error(session, 403, "No control permission");
            return;
        }
    }
    
    if (!msg.has_vr_control()) {
        send_error(session, 400, "Missing vr_control payload");
        return;
    }
    
    const auto& vr_intent = msg.vr_control();
    
    // 发布到 ROS2
    if (ros2_bridge_) {
        ros2_bridge_->publish_vr_intent(vr_intent);
    }
    
    // 构建 VRSystemState 并广播给订阅者
    if (server_) {
        VRSystemState vr_state;
        
        // 设置时间戳
        set_timestamp(vr_state.mutable_header()->mutable_stamp());
        vr_state.mutable_header()->set_frame_id("vr_system");
        
        // VR 连接状态
        vr_state.set_connected(VRSessionManager::instance().is_connected());
        
        // 头部位姿
        if (vr_intent.has_head_pose()) {
            *vr_state.mutable_head_pose() = vr_intent.head_pose();
        }
        
        // 控制器状态
        vr_state.set_left_controller_active(
            vr_intent.has_left_hand() && vr_intent.left_hand().active());
        vr_state.set_right_controller_active(
            vr_intent.has_right_hand() && vr_intent.right_hand().active());
        
        // Clutch 状态 (假设 grip > 0.5 表示 clutch engaged)
        vr_state.set_left_clutch_engaged(
            vr_intent.has_left_hand() && vr_intent.left_hand().grip() > 0.5f);
        vr_state.set_right_clutch_engaged(
            vr_intent.has_right_hand() && vr_intent.right_hand().grip() > 0.5f);
        
        // 序列化并缓存
        std::vector<uint8_t> state_data(vr_state.ByteSizeLong());
        vr_state.SerializeToArray(state_data.data(), static_cast<int>(state_data.size()));
        state_cache_.update_vr_system_state(state_data);
        
        // 包装为 WebSocketMessage 并广播
        WebSocketMessage ws_msg;
        ws_msg.set_type(MSG_VR_SYSTEM_STATE);
        set_timestamp(ws_msg.mutable_timestamp());
        *ws_msg.mutable_vr_system_state() = vr_state;
        
        std::vector<uint8_t> broadcast_data(ws_msg.ByteSizeLong());
        ws_msg.SerializeToArray(broadcast_data.data(), static_cast<int>(broadcast_data.size()));
        
        server_->broadcast_to_subscribers("vr_system_state", broadcast_data);
    }
}

void MessageHandler::handle_chassis_velocity(std::shared_ptr<Session> session,
                                              const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_chassis_velocity()) {
        send_error(session, 400, "Missing chassis_velocity payload");
        return;
    }
    
    if (ros2_bridge_) {
        const auto& vel = msg.chassis_velocity();
        ros2_bridge_->publish_cmd_vel(vel.linear_x(), vel.linear_y(), vel.angular_z());
    }
}

void MessageHandler::handle_joint_command(std::shared_ptr<Session> session,
                                           const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_joint_command()) {
        send_error(session, 400, "Missing joint_command payload");
        return;
    }
    
    if (ros2_bridge_) {
        const auto& cmd = msg.joint_command();
        ros2_bridge_->publish_joint_command(cmd);
    }
}

void MessageHandler::handle_end_effector_command(std::shared_ptr<Session> session,
                                                  const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_end_effector_cmd()) {
        send_error(session, 400, "Missing end_effector_cmd payload");
        return;
    }
    
    if (ros2_bridge_) {
        const auto& cmd = msg.end_effector_cmd();
        ros2_bridge_->publish_end_effector_command(cmd);
    }
}

void MessageHandler::handle_gripper_command(std::shared_ptr<Session> session,
                                             const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_gripper_command()) {
        send_error(session, 400, "Missing gripper_command payload");
        return;
    }
    
    if (ros2_bridge_) {
        const auto& cmd = msg.gripper_command();
        ros2_bridge_->publish_gripper_command(cmd);
    }
}

void MessageHandler::handle_navigation_goal(std::shared_ptr<Session> session,
                                             const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_navigation_goal()) {
        send_error(session, 400, "Missing navigation_goal payload");
        return;
    }
    
    if (ros2_bridge_) {
        const auto& goal = msg.navigation_goal();
        ros2_bridge_->publish_navigation_goal(goal);
    }
}

void MessageHandler::handle_emergency_stop(std::shared_ptr<Session> session,
                                            const WebSocketMessage& msg) {
    // 紧急停止不需要控制权检查，安全优先
    // 任何认证用户都可以触发急停
    
    std::cout << "[MessageHandler] Emergency stop triggered by session " 
              << session->session_id() << std::endl;
    
    // 1. 立即发布零速度到底盘
    if (ros2_bridge_) {
        ros2_bridge_->publish_cmd_vel(0.0, 0.0, 0.0);
    }
    
    // 2. 触发看门狗的紧急停止逻辑 (会发布到 /emergency_stop 话题)
    if (watchdog_) {
        watchdog_->trigger_emergency_stop();
    }
    
    // 3. 构建并广播急停通知给所有订阅者
    if (server_) {
        WebSocketMessage notification;
        notification.set_type(MSG_EMERGENCY_STOP);
        set_timestamp(notification.mutable_timestamp());
        
        auto* estop = notification.mutable_emergency_stop();
        set_timestamp(estop->mutable_header()->mutable_stamp());
        estop->set_active(true);
        estop->set_source("websocket");
        estop->set_reason("User triggered emergency stop");
        
        std::vector<uint8_t> data(notification.ByteSizeLong());
        notification.SerializeToArray(data.data(), static_cast<int>(data.size()));
        
        // 广播给所有连接的客户端 (不仅仅是订阅者)
        server_->broadcast(data);
    }
}

void MessageHandler::send_error(std::shared_ptr<Session> session,
                                 int32_t code,
                                 const std::string& message) {
    qyh::dataplane::WebSocketMessage response;
    response.set_type(qyh::dataplane::MSG_ERROR);
    
    auto* error = response.mutable_error();
    error->set_code(code);
    error->set_message(message);
    
    set_timestamp(response.mutable_timestamp());
    
    std::vector<uint8_t> data(response.ByteSizeLong());
    response.SerializeToArray(data.data(), static_cast<int>(data.size()));
    session->send(data);
}

void MessageHandler::send_auth_response(std::shared_ptr<Session> session,
                                         bool success,
                                         const std::string& error_message) {
    qyh::dataplane::WebSocketMessage response;
    response.set_type(qyh::dataplane::MSG_AUTH_RESPONSE);
    
    auto* auth_resp = response.mutable_auth_response();
    auth_resp->set_success(success);
    
    if (!success) {
        auto* error = auth_resp->mutable_error();
        error->set_code(401);
        error->set_message(error_message);
    } else {
        // 填充会话信息
        auto* session_info = auth_resp->mutable_session();
        session_info->set_session_id(session->session_id());
        
        if (session->has_user_info()) {
            auto* user = session_info->mutable_user();
            const auto& user_info = session->user_info();
            user->set_user_id(user_info.user_id);
            user->set_username(user_info.username);
            user->set_role(user_info.role);
            for (const auto& perm : user_info.permissions) {
                user->add_permissions(perm);
            }
        }
        
        set_timestamp(session_info->mutable_connected_at());
        session_info->set_has_control(session->has_control_permission());
    }
    
    set_timestamp(response.mutable_timestamp());
    
    std::vector<uint8_t> data(response.ByteSizeLong());
    response.SerializeToArray(data.data(), static_cast<int>(data.size()));
    std::cout << "[MessageHandler] 发送认证响应, 大小: " << data.size() << " 字节, success=" << success << std::endl;
    session->send(data);
}

void MessageHandler::set_timestamp(qyh::dataplane::Timestamp* timestamp) {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000;
    
    timestamp->set_seconds(seconds);
    timestamp->set_nanos(static_cast<int32_t>(nanos));
}

void MessageHandler::on_disconnect(const std::string& session_id) {
    if (watchdog_) {
        watchdog_->unregister(session_id);
    }
}

// ==================== 新增消息处理函数 ====================

void MessageHandler::handle_navigation_control(std::shared_ptr<Session> session,
                                                const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_navigation_control()) {
        send_error(session, 400, "Missing navigation_control payload");
        return;
    }
    
    const auto& ctrl = msg.navigation_control();
    const std::string& action = ctrl.action();
    
    std::cout << "[MessageHandler] Navigation control: " << action 
              << " from session " << session->session_id() << std::endl;
    
    if (ros2_bridge_) {
        if (action == "cancel") {
            ros2_bridge_->cancel_navigation();
        } else if (action == "pause") {
            ros2_bridge_->pause_navigation();
        } else if (action == "resume") {
            ros2_bridge_->resume_navigation();
        } else {
            send_error(session, 400, "Unknown navigation action: " + action);
        }
    }
}

void MessageHandler::handle_lift_command(std::shared_ptr<Session> session,
                                          const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_lift_command()) {
        send_error(session, 400, "Missing lift_command payload");
        return;
    }
    
    if (ros2_bridge_) {
        const auto& cmd = msg.lift_command();
        ros2_bridge_->publish_lift_command(cmd);
    }
}

void MessageHandler::handle_waist_command(std::shared_ptr<Session> session,
                                           const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_waist_command()) {
        send_error(session, 400, "Missing waist_command payload");
        return;
    }
    
    if (ros2_bridge_) {
        const auto& cmd = msg.waist_command();
        ros2_bridge_->publish_waist_command(cmd);
    }
}

void MessageHandler::handle_head_command(std::shared_ptr<Session> session,
                                          const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_head_command()) {
        send_error(session, 400, "Missing head_command payload");
        return;
    }
    
    if (ros2_bridge_) {
        const auto& cmd = msg.head_command();
        ros2_bridge_->publish_head_command(cmd);
    }
}

void MessageHandler::handle_arm_move(std::shared_ptr<Session> session,
                                      const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_arm_move()) {
        send_error(session, 400, "Missing arm_move payload");
        return;
    }
    
    if (ros2_bridge_) {
        const auto& cmd = msg.arm_move();
        ros2_bridge_->publish_arm_move_command(cmd);
    }
}

void MessageHandler::handle_arm_jog(std::shared_ptr<Session> session,
                                     const WebSocketMessage& msg) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_arm_jog()) {
        send_error(session, 400, "Missing arm_jog payload");
        return;
    }
    
    if (ros2_bridge_) {
        const auto& cmd = msg.arm_jog();
        ros2_bridge_->publish_arm_jog_command(cmd);
    }
}

} // namespace qyh::dataplane
