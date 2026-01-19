/**
 * @file message_handler.cpp
 * @brief 消息处理器实现
 */

#include "data_plane/message_handler.hpp"
#include "data_plane/session.hpp"
#include "data_plane/auth.hpp"
#include "data_plane/state_cache.hpp"
#include "data_plane/config.hpp"

#ifdef WITH_ROS2
#include "data_plane/ros2_bridge.hpp"
#include "data_plane/watchdog.hpp"
#endif

// Protobuf 生成的头文件
#include "messages.pb.h"

#include <iostream>

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
    qyh::proto::WebSocketMessage msg;
    if (!msg.ParseFromArray(data.data(), static_cast<int>(data.size()))) {
        std::cerr << "Failed to parse WebSocketMessage" << std::endl;
        send_error(session, 400, "Invalid message format");
        return;
    }
    
    // 检查鉴权状态
    if (session->state() == SessionState::CONNECTING) {
        // 未认证时只能发送认证请求
        if (msg.type() != qyh::proto::MessageType::AUTH_REQUEST) {
            send_error(session, 401, "Authentication required");
            return;
        }
    }
    
    // 根据消息类型分发处理
    switch (msg.type()) {
        case qyh::proto::MessageType::AUTH_REQUEST:
            handle_auth_request(session, data);
            break;
            
        case qyh::proto::MessageType::SUBSCRIBE_REQUEST:
            handle_subscribe(session, data);
            break;
            
        case qyh::proto::MessageType::HEARTBEAT:
            handle_heartbeat(session, data);
            break;
            
        case qyh::proto::MessageType::VR_CONTROL_INTENT:
            handle_vr_control(session, data);
            break;
            
        case qyh::proto::MessageType::CHASSIS_VELOCITY:
            handle_chassis_velocity(session, data);
            break;
            
        case qyh::proto::MessageType::JOINT_COMMAND:
            handle_joint_command(session, data);
            break;
            
        default:
            send_error(session, 400, "Unknown message type");
            break;
    }
}

void MessageHandler::handle_auth_request(std::shared_ptr<Session> session,
                                          const std::vector<uint8_t>& /*payload*/) {
    qyh::proto::WebSocketMessage msg;
    // TODO: 从 payload 解析 AuthRequest
    
    // 如果禁用认证，直接通过
    if (!config_.auth.enabled) {
        session->set_state(SessionState::AUTHENTICATED);
        send_auth_response(session, true);
        return;
    }
    
    // TODO: 从 msg.auth_request().token() 获取 token
    std::string token; // = msg.auth_request().token();
    
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
    
    // 认证成功
    session->set_user_info(*user_info);
    session->set_state(SessionState::AUTHENTICATED);
    
    std::cout << "Session " << session->session_id() 
              << " authenticated as " << user_info->username << std::endl;
    
    send_auth_response(session, true);
}

void MessageHandler::handle_subscribe(std::shared_ptr<Session> session,
                                       const std::vector<uint8_t>& /*payload*/) {
    // TODO: 解析 SubscribeRequest
    // qyh::proto::SubscribeRequest req;
    
    // for (const auto& topic : req.topics()) {
    //     session->subscribe(topic);
    // }
    
    session->set_state(SessionState::ACTIVE);
}

void MessageHandler::handle_heartbeat(std::shared_ptr<Session> session,
                                       const std::vector<uint8_t>& /*payload*/) {
    session->update_heartbeat();
    
#ifdef WITH_ROS2
    if (watchdog_) {
        watchdog_->feed(session->session_id());
    }
#endif
    
    // 回复心跳
    qyh::proto::WebSocketMessage response;
    response.set_type(qyh::proto::MessageType::HEARTBEAT);
    
    auto* header = response.mutable_header();
    auto now = std::chrono::system_clock::now();
    auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(
        now.time_since_epoch()).count();
    auto now_nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count() % 1000000000;
    header->mutable_timestamp()->set_sec(static_cast<int32_t>(now_sec));
    header->mutable_timestamp()->set_nsec(static_cast<int32_t>(now_nsec));
    
    std::vector<uint8_t> data(response.ByteSizeLong());
    response.SerializeToArray(data.data(), static_cast<int>(data.size()));
    session->send(data);
}

void MessageHandler::handle_vr_control(std::shared_ptr<Session> session,
                                        const std::vector<uint8_t>& payload) {
    // 检查控制权限
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
#ifdef WITH_ROS2
    if (ros2_bridge_) {
        ros2_bridge_->publish_vr_intent(payload);
    }
#else
    (void)payload;
#endif
}

void MessageHandler::handle_chassis_velocity(std::shared_ptr<Session> session,
                                              const std::vector<uint8_t>& /*payload*/) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    // TODO: 解析 ChassisVelocity 并发布
#ifdef WITH_ROS2
    if (ros2_bridge_) {
        // qyh::proto::ChassisVelocity vel;
        // ros2_bridge_->publish_cmd_vel(vel.linear_x(), vel.linear_y(), vel.angular_z());
    }
#endif
}

void MessageHandler::handle_joint_command(std::shared_ptr<Session> session,
                                           const std::vector<uint8_t>& /*payload*/) {
    if (!session->has_control_permission()) {
        send_error(session, 403, "No control permission");
        return;
    }
    
    // TODO: 发布关节命令
}

void MessageHandler::send_error(std::shared_ptr<Session> session,
                                 int32_t code,
                                 const std::string& message) {
    qyh::proto::WebSocketMessage response;
    response.set_type(qyh::proto::MessageType::ERROR);
    
    auto* error = response.mutable_error();
    error->set_code(code);
    error->set_message(message);
    
    std::vector<uint8_t> data(response.ByteSizeLong());
    response.SerializeToArray(data.data(), static_cast<int>(data.size()));
    session->send(data);
}

void MessageHandler::send_auth_response(std::shared_ptr<Session> session,
                                         bool success,
                                         const std::string& error_message) {
    qyh::proto::WebSocketMessage response;
    response.set_type(qyh::proto::MessageType::AUTH_RESPONSE);
    
    auto* auth_resp = response.mutable_auth_response();
    auth_resp->set_success(success);
    if (!success) {
        auth_resp->set_error_message(error_message);
    } else {
        auth_resp->set_session_id(session->session_id());
    }
    
    std::vector<uint8_t> data(response.ByteSizeLong());
    response.SerializeToArray(data.data(), static_cast<int>(data.size()));
    session->send(data);
}

} // namespace qyh::dataplane
