/**
 * @file message_handler.hpp
 * @brief 消息处理器
 * 
 * 负责解析和处理所有入站 WebSocket 消息
 */

#pragma once

#include <memory>
#include <vector>
#include <cstdint>
#include <functional>
#include <string>

// 前向声明 Protobuf 类型
namespace qyh::dataplane {
    class WebSocketMessage;
    class Timestamp;
}

namespace qyh::dataplane {

// 前向声明
class Session;
class JWTValidator;
class StateCache;
class Config;
class ControlSyncService;
class Server;

class ROS2Bridge;
class Watchdog;

/**
 * @brief 消息处理器
 * 
 * 负责解析和处理所有入站 WebSocket 消息
 */
class MessageHandler {
public:
    /**
     * @brief 构造函数
     * @param config 配置
     * @param validator JWT 验证器
     * @param state_cache 状态缓存
     */
    MessageHandler(const Config& config,
                   JWTValidator& validator,
                   StateCache& state_cache);
    
    /**
     * @brief 设置 ROS2 桥接
     */
    void set_ros2_bridge(ROS2Bridge* bridge) { ros2_bridge_ = bridge; }
    
    /**
     * @brief 设置 WebSocket 服务器 (用于广播 VR 状态)
     */
    void set_server(Server* server) { server_ = server; }
    
    /**
     * @brief 设置 Watchdog
     */
    void set_watchdog(Watchdog* watchdog) { watchdog_ = watchdog; }

    /**
     * @brief 处理入站消息
     * @param session 会话
     * @param data 消息数据
     */
    void handle_message(std::shared_ptr<Session> session,
                        const std::vector<uint8_t>& data);

    /**
     * @brief 设置控制权同步服务
     */
    void set_control_sync(ControlSyncService* service) { control_sync_ = service; }

    /**
     * @brief 处理会话断开后的清理
     */
    void on_disconnect(const std::string& session_id);
    
private:
    // ==================== 消息处理函数 ====================
    
    /** @brief 处理认证请求 */
    void handle_auth_request(std::shared_ptr<Session> session,
                             const WebSocketMessage& msg);
    
    /** @brief 处理订阅请求 */
    void handle_subscribe(std::shared_ptr<Session> session,
                          const WebSocketMessage& msg);
    
    /** @brief 处理取消订阅 */
    void handle_unsubscribe(std::shared_ptr<Session> session,
                            const WebSocketMessage& msg);
    
    /** @brief 处理心跳 */
    void handle_heartbeat(std::shared_ptr<Session> session,
                          const WebSocketMessage& msg);
    
    /** @brief 处理 VR 控制意图 */
    void handle_vr_control(std::shared_ptr<Session> session,
                           const WebSocketMessage& msg);
    
    /** @brief 处理底盘速度命令 */
    void handle_chassis_velocity(std::shared_ptr<Session> session,
                                 const WebSocketMessage& msg);
    
    /** @brief 处理关节命令 */
    void handle_joint_command(std::shared_ptr<Session> session,
                              const WebSocketMessage& msg);
    
    /** @brief 处理末端执行器命令 */
    void handle_end_effector_command(std::shared_ptr<Session> session,
                                     const WebSocketMessage& msg);
    
    /** @brief 处理夹爪命令 */
    void handle_gripper_command(std::shared_ptr<Session> session,
                                const WebSocketMessage& msg);
    
    /** @brief 处理导航目标 */
    void handle_navigation_goal(std::shared_ptr<Session> session,
                                const WebSocketMessage& msg);
    
    /** @brief 处理紧急停止 */
    void handle_emergency_stop(std::shared_ptr<Session> session,
                               const WebSocketMessage& msg);
    
    /** @brief 处理导航控制 (取消/暂停/恢复) */
    void handle_navigation_control(std::shared_ptr<Session> session,
                                   const WebSocketMessage& msg);
    
    /** @brief 处理升降控制 */
    void handle_lift_command(std::shared_ptr<Session> session,
                             const WebSocketMessage& msg);
    
    /** @brief 处理腰部控制 */
    void handle_waist_command(std::shared_ptr<Session> session,
                              const WebSocketMessage& msg);
    
    /** @brief 处理头部控制 */
    void handle_head_command(std::shared_ptr<Session> session,
                             const WebSocketMessage& msg);
    
    /** @brief 处理机械臂运动命令 (MoveJ/MoveL) */
    void handle_arm_move(std::shared_ptr<Session> session,
                         const WebSocketMessage& msg);
    
    /** @brief 处理机械臂点动命令 (Jog) */
    void handle_arm_jog(std::shared_ptr<Session> session,
                        const WebSocketMessage& msg);
    
    // ==================== 响应发送函数 ====================
    
    /** @brief 发送错误响应 */
    void send_error(std::shared_ptr<Session> session,
                    int32_t code,
                    const std::string& message);
    
    /** @brief 发送认证响应 */
    void send_auth_response(std::shared_ptr<Session> session,
                            bool success,
                            const std::string& error_message = "");
    
    // ==================== 辅助函数 ====================
    
    /** @brief 设置时间戳 */
    void set_timestamp(qyh::dataplane::Timestamp* timestamp);
    
private:
    const Config& config_;
    JWTValidator& validator_;
    StateCache& state_cache_;
    ControlSyncService* control_sync_ = nullptr;
    Server* server_ = nullptr;
    
    ROS2Bridge* ros2_bridge_ = nullptr;
    Watchdog* watchdog_ = nullptr;
};

} // namespace qyh::dataplane
