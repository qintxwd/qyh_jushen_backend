/**
 * @file state_cache.cpp
 * @brief 状态缓存实现
 * 
 * 线程安全的状态数据缓存，支持双臂机器人的完整状态管理
 */

#include "data_plane/state_cache.hpp"

namespace qyh::dataplane {

// ==================== 综合状态 ====================

void StateCache::update_robot_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_robot_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (robot_state_.empty()) {
        return std::nullopt;
    }
    return robot_state_;
}

// ==================== 关节状态 ====================

void StateCache::update_joint_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    joint_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_joint_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (joint_state_.empty()) {
        return std::nullopt;
    }
    return joint_state_;
}

// ==================== 机械臂状态 ====================

void StateCache::update_arm_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    arm_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_arm_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (arm_state_.empty()) {
        return std::nullopt;
    }
    return arm_state_;
}

void StateCache::update_left_arm_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    left_arm_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_left_arm_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (left_arm_state_.empty()) {
        return std::nullopt;
    }
    return left_arm_state_;
}

void StateCache::update_right_arm_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    right_arm_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_right_arm_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (right_arm_state_.empty()) {
        return std::nullopt;
    }
    return right_arm_state_;
}

// ==================== 底盘状态 ====================

void StateCache::update_chassis_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    chassis_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_chassis_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (chassis_state_.empty()) {
        return std::nullopt;
    }
    return chassis_state_;
}

// ==================== 执行器状态 ====================

void StateCache::update_lift_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    lift_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_lift_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (lift_state_.empty()) {
        return std::nullopt;
    }
    return lift_state_;
}

void StateCache::update_waist_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    waist_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_waist_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (waist_state_.empty()) {
        return std::nullopt;
    }
    return waist_state_;
}

void StateCache::update_head_pan_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    head_pan_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_head_pan_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (head_pan_state_.empty()) {
        return std::nullopt;
    }
    return head_pan_state_;
}

void StateCache::update_head_tilt_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    head_tilt_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_head_tilt_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (head_tilt_state_.empty()) {
        return std::nullopt;
    }
    return head_tilt_state_;
}

// ==================== 夹爪状态 ====================

void StateCache::update_gripper_state(const std::string& gripper_id, const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    gripper_states_[gripper_id] = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_gripper_state(const std::string& gripper_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = gripper_states_.find(gripper_id);
    if (it == gripper_states_.end() || it->second.empty()) {
        return std::nullopt;
    }
    return it->second;
}

// ==================== 通用接口 ====================

bool StateCache::is_stale(std::chrono::milliseconds max_age) const {
    auto now = std::chrono::steady_clock::now();
    auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time_);
    return age > max_age;
}

void StateCache::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_state_.clear();
    joint_state_.clear();
    arm_state_.clear();
    left_arm_state_.clear();
    right_arm_state_.clear();
    chassis_state_.clear();
    lift_state_.clear();
    waist_state_.clear();
    head_pan_state_.clear();
    head_tilt_state_.clear();
    gripper_states_.clear();
}

} // namespace qyh::dataplane
