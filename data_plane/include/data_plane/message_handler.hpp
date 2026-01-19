/**
 * @file message_handler.hpp
 * @brief 消息处理器
 */

#pragma once

#include <memory>
#include <vector>
#include <cstdint>
#include <functional>

namespace qyh::dataplane {

// 前向声明
class Session;
class JWTValidator;
class StateCache;
class Config;

#ifdef WITH_ROS2
class ROS2Bridge;
class Watchdog;
#endif

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
    
#ifdef WITH_ROS2
    /**
     * @brief 设置 ROS2 桥接
     */
    void set_ros2_bridge(ROS2Bridge* bridge) { ros2_bridge_ = bridge; }
    
    /**
     * @brief 设置 Watchdog
     */
    void set_watchdog(Watchdog* watchdog) { watchdog_ = watchdog; }
#endif
    
    /**
     * @brief 处理入站消息
     * @param session 会话
     * @param data 消息数据
     */
    void handle_message(std::shared_ptr<Session> session,
                        const std::vector<uint8_t>& data);
    
private:
    /**
     * @brief 处理认证请求
     */
    void handle_auth_request(std::shared_ptr<Session> session,
                             const std::vector<uint8_t>& payload);
    
    /**
     * @brief 处理订阅请求
     */
    void handle_subscribe(std::shared_ptr<Session> session,
                          const std::vector<uint8_t>& payload);
    
    /**
     * @brief 处理取消订阅
     */
    void handle_unsubscribe(std::shared_ptr<Session> session,
                            const std::vector<uint8_t>& payload);
    
    /**
     * @brief 处理心跳
     */
    void handle_heartbeat(std::shared_ptr<Session> session,
                          const std::vector<uint8_t>& payload);
    
    /**
     * @brief 处理 VR 控制意图
     */
    void handle_vr_control(std::shared_ptr<Session> session,
                           const std::vector<uint8_t>& payload);
    
    /**
     * @brief 处理底盘速度命令
     */
    void handle_chassis_velocity(std::shared_ptr<Session> session,
                                 const std::vector<uint8_t>& payload);
    
    /**
     * @brief 处理关节命令
     */
    void handle_joint_command(std::shared_ptr<Session> session,
                              const std::vector<uint8_t>& payload);
    
    /**
     * @brief 发送错误响应
     */
    void send_error(std::shared_ptr<Session> session,
                    int32_t code,
                    const std::string& message);
    
    /**
     * @brief 发送认证响应
     */
    void send_auth_response(std::shared_ptr<Session> session,
                            bool success,
                            const std::string& error_message = "");
    
private:
    const Config& config_;
    JWTValidator& validator_;
    StateCache& state_cache_;
    
#ifdef WITH_ROS2
    ROS2Bridge* ros2_bridge_ = nullptr;
    Watchdog* watchdog_ = nullptr;
#endif
};

} // namespace qyh::dataplane
