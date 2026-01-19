/**
 * @file state_cache.cpp
 * @brief 状态缓存实现
 */

#include "data_plane/state_cache.hpp"

namespace qyh::dataplane {

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

} // namespace qyh::dataplane
