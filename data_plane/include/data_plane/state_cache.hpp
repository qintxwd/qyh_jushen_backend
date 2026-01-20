/**
 * @file state_cache.hpp
 * @brief 状态缓存
 * 
 * 缓存从 ROS2 接收的最新状态，供 WebSocket 推送使用
 * 支持双臂机器人的完整状态管理
 */

#pragma once

#include <mutex>
#include <chrono>
#include <optional>
#include <vector>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <memory>

namespace qyh::dataplane {

/**
 * @brief 状态缓存
 * 
 * 线程安全的状态数据缓存，支持按类型存取最新状态
 */
class StateCache {
public:
    StateCache() = default;
    
    // ==================== 综合状态 ====================
    
    /** @brief 更新机器人综合状态 */
    void update_robot_state(std::shared_ptr<const std::vector<uint8_t>> data);
    void update_robot_state(const std::vector<uint8_t>& data);
    
    /** @brief 获取机器人综合状态 */
    std::shared_ptr<const std::vector<uint8_t>> get_robot_state() const;
    
    // ==================== 关节状态 ====================
    
    /** @brief 更新关节状态 */
    void update_joint_state(std::shared_ptr<const std::vector<uint8_t>> data);
    void update_joint_state(const std::vector<uint8_t>& data);
    
    /** @brief 获取关节状态 */
    std::shared_ptr<const std::vector<uint8_t>> get_joint_state() const;
    
    // ==================== 机械臂状态 ====================
    
    /** @brief 更新机械臂综合状态 */
    void update_arm_state(const std::vector<uint8_t>& data);
    void update_arm_state(std::shared_ptr<const std::vector<uint8_t>> data);
    
    /** @brief 获取机械臂综合状态 */
    std::shared_ptr<const std::vector<uint8_t>> get_arm_state() const;
    
    /** @brief 更新左臂关节状态 */
    void update_left_arm_state(const std::vector<uint8_t>& data);
    void update_left_arm_state(std::shared_ptr<const std::vector<uint8_t>> data);
    
    /** @brief 获取左臂关节状态 */
    std::shared_ptr<const std::vector<uint8_t>> get_left_arm_state() const;
    
    /** @brief 更新右臂关节状态 */
    void update_right_arm_state(const std::vector<uint8_t>& data);
    void update_right_arm_state(std::shared_ptr<const std::vector<uint8_t>> data);
    
    /** @brief 获取右臂关节状态 */
    std::shared_ptr<const std::vector<uint8_t>> get_right_arm_state() const;
    
    // ==================== 底盘状态 ====================
    
    /** @brief 更新底盘状态 */
    void update_chassis_state(const std::vector<uint8_t>& data);
    void update_chassis_state(std::shared_ptr<const std::vector<uint8_t>> data);
    
    /** @brief 获取底盘状态 */
    std::shared_ptr<const std::vector<uint8_t>> get_chassis_state() const;
    
    // ==================== 执行器状态 ====================
    
    /** @brief 更新升降状态 */
    void update_lift_state(const std::vector<uint8_t>& data);
    void update_lift_state(std::shared_ptr<const std::vector<uint8_t>> data);
    
    /** @brief 获取升降状态 */
    std::shared_ptr<const std::vector<uint8_t>> get_lift_state() const;
    
    /** @brief 更新腰部状态 */
    void update_waist_state(const std::vector<uint8_t>& data);
    void update_waist_state(std::shared_ptr<const std::vector<uint8_t>> data);
    
    /** @brief 获取腰部状态 */
    std::shared_ptr<const std::vector<uint8_t>> get_waist_state() const;
    
    /** @brief 更新头部Pan状态 */
    void update_head_pan_state(const std::vector<uint8_t>& data);
    void update_head_pan_state(std::shared_ptr<const std::vector<uint8_t>> data);
    
    /** @brief 获取头部Pan状态 */
    std::shared_ptr<const std::vector<uint8_t>> get_head_pan_state() const;

    /** @brief 更新头部Tilt状态 */
    void update_head_tilt_state(const std::vector<uint8_t>& data);
    void update_head_tilt_state(std::shared_ptr<const std::vector<uint8_t>> data);
    
    /** @brief 获取头部Tilt状态 */
    std::shared_ptr<const std::vector<uint8_t>> get_head_tilt_state() const;
    
    // ==================== 夹爪状态 ====================
    
    /** @brief 更新夹爪状态 */
    void update_gripper_state(const std::string& gripper_id, const std::vector<uint8_t>& data);
    
    /** @brief 获取夹爪状态 */
    std::optional<std::vector<uint8_t>> get_gripper_state(const std::string& gripper_id) const;
    
    // ==================== VR 状态 ====================
    
    /** @brief 更新 VR 系统状态 */
    void update_vr_system_state(const std::vector<uint8_t>& data);
    
    /** @brief 获取 VR 系统状态 */
    std::optional<std::vector<uint8_t>> get_vr_system_state() const;
    
    // ==================== 通用接口 ====================
    
    /** @brief 获取状态更新时间 */
    std::chrono::steady_clock::time_point last_update_time() const {
        return last_update_time_;
    }
    
    /** @brief 检查状态是否过期 */
    bool is_stale(std::chrono::milliseconds max_age) const;
    
    /** @brief 清空所有缓存 */
    void clear();
    
private:
    mutable std::mutex mutex_;
    
    // 状态数据 (使用 shared_ptr 减少拷贝)
    std::shared_ptr<const std::vector<uint8_t>> robot_state_;
    std::shared_ptr<const std::vector<uint8_t>> joint_state_;
    std::shared_ptr<const std::vector<uint8_t>> arm_state_;
    std::shared_ptr<const std::vector<uint8_t>> left_arm_state_;
    std::shared_ptr<const std::vector<uint8_t>> right_arm_state_;
    std::shared_ptr<const std::vector<uint8_t>> chassis_state_;
    std::shared_ptr<const std::vector<uint8_t>> lift_state_;
    std::shared_ptr<const std::vector<uint8_t>> waist_state_;
    std::shared_ptr<const std::vector<uint8_t>> head_pan_state_;
    std::shared_ptr<const std::vector<uint8_t>> head_tilt_state_;
    
    // 复杂类型如 map 仍需拷贝或使用 atomic_shared_ptr，此处保持原样或仅值类型优化
    std::unordered_map<std::string, std::vector<uint8_t>> gripper_states_;
    std::vector<uint8_t> vr_system_state_;
    
    std::chrono::steady_clock::time_point last_update_time_;
};

} // namespace qyh::dataplane
