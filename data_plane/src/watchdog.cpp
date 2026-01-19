/**
 * @file watchdog.cpp
 * @brief Watchdog 安全监控实现
 * 
 * 根据重构.md要求实现 200ms 超时机制
 */

#include "data_plane/watchdog.hpp"
#include "data_plane/config.hpp"
#include "data_plane/ros2_bridge.hpp"

#include <iostream>

namespace qyh::dataplane {

Watchdog::Watchdog(const Config& config, ROS2Bridge& ros2_bridge)
    : config_(config)
    , ros2_bridge_(ros2_bridge)
    , timeout_(config.watchdog.timeout_ms)
    , check_interval_(config.watchdog.check_interval_ms)
{
}

Watchdog::~Watchdog() {
    stop();
}

void Watchdog::start() {
    if (running_) {
        return;
    }
    
    running_ = true;
    emergency_triggered_ = false;
    
    monitor_thread_ = std::thread(&Watchdog::monitor_thread, this);
    
    std::cout << "Watchdog started (timeout: " << timeout_.count() << "ms)" << std::endl;
}

void Watchdog::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
    
    std::cout << "Watchdog stopped" << std::endl;
}

void Watchdog::feed(const std::string& session_id) {
    std::lock_guard<std::mutex> lock(heartbeats_mutex_);
    heartbeats_[session_id] = std::chrono::steady_clock::now();
    
    // 收到心跳后重置紧急停止状态
    emergency_triggered_ = false;
    
    // 发布 ROS2 心跳
    ros2_bridge_.publish_watchdog_heartbeat();
}

void Watchdog::unregister(const std::string& session_id) {
    std::lock_guard<std::mutex> lock(heartbeats_mutex_);
    heartbeats_.erase(session_id);
}

bool Watchdog::has_active_sessions() const {
    std::lock_guard<std::mutex> lock(heartbeats_mutex_);
    return !heartbeats_.empty();
}

void Watchdog::monitor_thread() {
    while (running_) {
        std::this_thread::sleep_for(check_interval_);
        
        if (!running_) {
            break;
        }
        
        auto now = std::chrono::steady_clock::now();
        bool all_timeout = true;
        
        {
            std::lock_guard<std::mutex> lock(heartbeats_mutex_);
            
            // 如果没有活跃会话，不触发超时
            if (heartbeats_.empty()) {
                all_timeout = false;
            } else {
                // 检查所有会话是否都超时
                for (auto it = heartbeats_.begin(); it != heartbeats_.end();) {
                    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - it->second);
                    
                    if (elapsed < timeout_) {
                        all_timeout = false;
                        ++it;
                    } else {
                        // 单个会话超时，移除它
                        std::cout << "Session " << it->first 
                                  << " heartbeat timeout (" << elapsed.count() << "ms)" 
                                  << std::endl;
                        it = heartbeats_.erase(it);
                    }
                }
            }
        }
        
        // 如果所有会话都超时，触发紧急停止
        if (all_timeout && !emergency_triggered_) {
            trigger_emergency_stop();
        }
    }
}

void Watchdog::trigger_emergency_stop() {
    // 防止重复触发
    if (emergency_triggered_.exchange(true)) {
        return;
    }
    
    std::cerr << "!!! WATCHDOG TIMEOUT - TRIGGERING EMERGENCY STOP !!!" << std::endl;
    
    // 发送零速度命令
    ros2_bridge_.publish_cmd_vel(0.0, 0.0, 0.0);
    
    // 调用用户回调
    if (emergency_stop_callback_) {
        emergency_stop_callback_();
    }
}

} // namespace qyh::dataplane
