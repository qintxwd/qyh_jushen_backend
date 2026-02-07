/**
 * @file state_cache.cpp
 * @brief 状态缓存实现
 * 
 * 线程安全的状态数据缓存，支持双臂机器人的完整状态管理
 */

#include "data_plane/state_cache.hpp"

namespace qyh::dataplane {

// ==================== 综合状态 ====================

void StateCache::update_robot_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_robot_state(const std::vector<uint8_t>& data) {
    update_robot_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_robot_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return robot_state_;
}

// ==================== 关节状态 ====================

void StateCache::update_joint_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    joint_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_joint_state(const std::vector<uint8_t>& data) {
    update_joint_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_joint_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return joint_state_;
}

// ==================== 机械臂状态 ====================

void StateCache::update_arm_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    arm_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_arm_state(const std::vector<uint8_t>& data) {
    update_arm_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_arm_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_state_;
}

void StateCache::update_left_arm_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    left_arm_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_left_arm_state(const std::vector<uint8_t>& data) {
    update_left_arm_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_left_arm_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return left_arm_state_;
}

void StateCache::update_right_arm_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    right_arm_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_right_arm_state(const std::vector<uint8_t>& data) {
    update_right_arm_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_right_arm_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return right_arm_state_;
}

// ==================== 底盘状态 ====================

void StateCache::update_chassis_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    chassis_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_chassis_state(const std::vector<uint8_t>& data) {
    update_chassis_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_chassis_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return chassis_state_;
}

// ==================== 执行器状态 ====================

void StateCache::update_lift_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    lift_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_lift_state(const std::vector<uint8_t>& data) {
    update_lift_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_lift_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lift_state_;
}

void StateCache::update_waist_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    waist_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_waist_state(const std::vector<uint8_t>& data) {
    update_waist_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_waist_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return waist_state_;
}

void StateCache::update_head_pan_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    head_pan_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_head_pan_state(const std::vector<uint8_t>& data) {
    update_head_pan_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_head_pan_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return head_pan_state_;
}

void StateCache::update_head_tilt_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    head_tilt_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_head_tilt_state(const std::vector<uint8_t>& data) {
    update_head_tilt_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_head_tilt_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
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

// ==================== VR 状态 ====================

void StateCache::update_vr_system_state(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    vr_system_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

std::optional<std::vector<uint8_t>> StateCache::get_vr_system_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (vr_system_state_.empty()) {
        return std::nullopt;
    }
    return vr_system_state_;
}

// ==================== 任务状态 ====================

void StateCache::update_task_state(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::lock_guard<std::mutex> lock(mutex_);
    task_state_ = data;
    last_update_time_ = std::chrono::steady_clock::now();
}

void StateCache::update_task_state(const std::vector<uint8_t>& data) {
    update_task_state(std::make_shared<std::vector<uint8_t>>(data));
}

std::shared_ptr<const std::vector<uint8_t>> StateCache::get_task_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return task_state_;
}

// ==================== 通用接口 ====================

bool StateCache::is_stale(std::chrono::milliseconds max_age) const {
    auto now = std::chrono::steady_clock::now();
    auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time_);
    return age > max_age;
}

void StateCache::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    robot_state_.reset();
    joint_state_.reset();
    arm_state_.reset();
    left_arm_state_.reset();
    right_arm_state_.reset();
    chassis_state_.reset();
    lift_state_.reset();
    waist_state_.reset();
    head_pan_state_.reset();
    head_tilt_state_.reset();
    task_state_.reset();
    gripper_states_.clear();
    vr_system_state_.clear();
}

} // namespace qyh::dataplane
