/**
 * @file watchdog.hpp
 * @brief Watchdog 安全监控模块
 * 
 * 根据重构.md要求，实现 200ms 超时机制
 */

#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <functional>
#include <thread>

namespace qyh::dataplane {

// 前向声明
class Config;
class ROS2Bridge;

/**
 * @brief Watchdog 监控器
 * 
 * 监控客户端心跳，超时后触发紧急停止
 */
class Watchdog {
public:
    using EmergencyStopCallback = std::function<void()>;
    
    /**
     * @brief 构造函数
     * @param config 配置
     * @param ros2_bridge ROS2 桥接（用于发送紧急停止）
     */
    Watchdog(const Config& config, ROS2Bridge& ros2_bridge);
    
    ~Watchdog();
    
    /**
     * @brief 启动 Watchdog
     */
    void start();
    
    /**
     * @brief 停止 Watchdog
     */
    void stop();
    
    /**
     * @brief 喂狗（收到客户端心跳时调用）
     * @param session_id 会话 ID
     */
    void feed(const std::string& session_id);
    
    /**
     * @brief 注销会话
     * @param session_id 会话 ID
     */
    void unregister(const std::string& session_id);
    
    /**
     * @brief 设置紧急停止回调
     */
    void set_emergency_stop_callback(EmergencyStopCallback callback) {
        emergency_stop_callback_ = std::move(callback);
    }
    
    /**
     * @brief 获取超时时间
     */
    std::chrono::milliseconds timeout() const { return timeout_; }
    
    /**
     * @brief 是否有活跃会话
     */
    bool has_active_sessions() const;
    
private:
    /**
     * @brief 监控线程
     */
    void monitor_thread();
    
    /**
     * @brief 触发紧急停止
     */
    void trigger_emergency_stop();
    
private:
    const Config& config_;
    ROS2Bridge& ros2_bridge_;
    
    std::chrono::milliseconds timeout_;
    std::chrono::milliseconds check_interval_;
    
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> heartbeats_;
    mutable std::mutex heartbeats_mutex_;
    
    std::atomic<bool> running_{false};
    std::thread monitor_thread_;
    
    EmergencyStopCallback emergency_stop_callback_;
    
    // 防止重复触发紧急停止
    std::atomic<bool> emergency_triggered_{false};
};

} // namespace qyh::dataplane
