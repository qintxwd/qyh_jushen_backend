/**
 * @file state_cache.hpp
 * @brief 状态缓存
 * 
 * 缓存从 ROS2 接收的最新状态，供 WebSocket 推送使用
 */

#pragma once

#include <mutex>
#include <chrono>
#include <optional>
#include <vector>
#include <cstdint>

namespace qyh::dataplane {

/**
 * @brief 状态缓存
 */
class StateCache {
public:
    StateCache() = default;
    
    /**
     * @brief 更新机器人状态
     * @param data 序列化的状态数据
     */
    void update_robot_state(const std::vector<uint8_t>& data);
    
    /**
     * @brief 获取机器人状态
     * @return 序列化的状态数据，如果没有则返回 std::nullopt
     */
    std::optional<std::vector<uint8_t>> get_robot_state() const;
    
    /**
     * @brief 更新关节状态
     */
    void update_joint_state(const std::vector<uint8_t>& data);
    
    /**
     * @brief 获取关节状态
     */
    std::optional<std::vector<uint8_t>> get_joint_state() const;
    
    /**
     * @brief 更新机械臂状态
     */
    void update_arm_state(const std::vector<uint8_t>& data);
    
    /**
     * @brief 获取机械臂状态
     */
    std::optional<std::vector<uint8_t>> get_arm_state() const;
    
    /**
     * @brief 更新底盘状态
     */
    void update_chassis_state(const std::vector<uint8_t>& data);
    
    /**
     * @brief 获取底盘状态
     */
    std::optional<std::vector<uint8_t>> get_chassis_state() const;
    
    /**
     * @brief 获取状态更新时间
     */
    std::chrono::steady_clock::time_point last_update_time() const {
        return last_update_time_;
    }
    
private:
    mutable std::mutex mutex_;
    
    std::vector<uint8_t> robot_state_;
    std::vector<uint8_t> joint_state_;
    std::vector<uint8_t> arm_state_;
    std::vector<uint8_t> chassis_state_;
    
    std::chrono::steady_clock::time_point last_update_time_;
};

} // namespace qyh::dataplane
