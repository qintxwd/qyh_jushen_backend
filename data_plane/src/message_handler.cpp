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

#ifdef WITH_ROS2
#include "data_plane/ros2_bridge.hpp"
#include "data_plane/watchdog.hpp"
#endif

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
    // 解析 WebSocketMessage
    qyh::dataplane::WebSocketMessage msg;
    if (!msg.ParseFromArray(data.data(), static_cast<int>(data.size()))) {
        std::cerr << "[MessageHandler] Failed to parse WebSocketMessage, size=" 
                  << data.size() << std::endl;
        send_error(session, 400, "Invalid message format");
        return;
    }
    
    // 检查鉴权状态
    if (session->state() == SessionState::CONNECTING) {
        // 未认证时只能发送认证请求
        if (msg.type() != qyh::dataplane::MSG_AUTH_REQUEST) {
            send_error(session, 401, "Authentication required");
            return;
        }
    }
    
    // 根据消息类型分发处理
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
            
        default:
            std::cerr << "[MessageHandler] Unknown message type: " 
                      << static_cast<int>(msg.type()) << std::endl;
            send_error(session, 400, "Unknown message type");
            break;
    }
}

void MessageHandler::handle_auth_request(std::shared_ptr<Session> session,
                                          const WebSocketMessage& msg) {
    // 从消息中提取认证请求
    if (!msg.has_auth_request()) {
        send_error(session, 400, "Missing auth_request payload");
        return;
    }
    
    const auto& auth_req = msg.auth_request();
    
    // 如果禁用认证，直接通过
    if (!config_.auth.enabled) {
        std::cout << "[MessageHandler] Auth disabled, auto-passing session " 
                  << session->session_id() << std::endl;
        session->set_state(SessionState::AUTHENTICATED);
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
    auto user_info = validator_.validate(token);
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
    session->set_state(SessionState::AUTHENTICATED);

    if (control_sync_) {
        control_sync_->associate_session(session->session_id(), user_info->user_id);
    }
    
    std::cout << "[MessageHandler] Session " << session->session_id() 
              << " authenticated as " << user_info->username 
              << " (" << auth_req.client_type() << " v" << auth_req.client_version() << ")"
              << std::endl;
    
    send_auth_response(session, true);
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
    
#ifdef WITH_ROS2
    if (watchdog_) {
        watchdog_->feed(session->session_id());
    }
#endif
    
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
    // 检查控制权限
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    if (!msg.has_vr_control()) {
        send_error(session, 400, "Missing vr_control payload");
        return;
    }
    
#ifdef WITH_ROS2
    if (ros2_bridge_) {
        const auto& vr_intent = msg.vr_control();
        ros2_bridge_->publish_vr_intent(vr_intent);
    }
#endif
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
    
#ifdef WITH_ROS2
    if (ros2_bridge_) {
        const auto& vel = msg.chassis_velocity();
        ros2_bridge_->publish_cmd_vel(vel.linear_x(), vel.linear_y(), vel.angular_z());
    }
#endif
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
    
#ifdef WITH_ROS2
    if (ros2_bridge_) {
        const auto& cmd = msg.joint_command();
        ros2_bridge_->publish_joint_command(cmd);
    }
#endif
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
    
#ifdef WITH_ROS2
    if (ros2_bridge_) {
        const auto& cmd = msg.end_effector_cmd();
        ros2_bridge_->publish_end_effector_command(cmd);
    }
#endif
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
    
#ifdef WITH_ROS2
    if (ros2_bridge_) {
        const auto& cmd = msg.gripper_command();
        ros2_bridge_->publish_gripper_command(cmd);
    }
#endif
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
    
#ifdef WITH_ROS2
    if (ros2_bridge_) {
        const auto& goal = msg.navigation_goal();
        ros2_bridge_->publish_navigation_goal(goal);
    }
#endif
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

} // namespace qyh::dataplane
