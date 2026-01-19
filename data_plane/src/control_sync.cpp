/**
 * @file control_sync.cpp
 * @brief 控制权同步服务实现
 */

#include "data_plane/control_sync.hpp"
#include <iostream>

// 简单的 HTTP 客户端（实际项目中可使用 cpr 或 Boost.Beast）
// 这里提供一个模拟实现，生产环境需要替换为真正的 HTTP 客户端

namespace qyh::dataplane {

// ==================== ControlSyncService ====================

ControlSyncService::ControlSyncService(const Config& config)
    : config_(config)
{
}

ControlSyncService::~ControlSyncService() {
    stop();
}

void ControlSyncService::start() {
    if (running_ || !config_.enabled) {
        return;
    }
    
    running_ = true;
    sync_thread_ = std::thread(&ControlSyncService::sync_loop, this);
    
    std::cout << "[ControlSync] Started with interval " 
              << config_.sync_interval_ms << "ms" << std::endl;
}

void ControlSyncService::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    if (sync_thread_.joinable()) {
        sync_thread_.join();
    }
    
    std::cout << "[ControlSync] Stopped" << std::endl;
}

void ControlSyncService::sync_now() {
    fetch_control_status();
}

ControlInfo ControlSyncService::get_control_info() const {
    std::lock_guard<std::mutex> lock(info_mutex_);
    return control_info_;
}

bool ControlSyncService::has_control(int64_t user_id) const {
    std::lock_guard<std::mutex> lock(info_mutex_);
    
    if (!control_info_.is_held) {
        return false;
    }
    
    if (control_info_.is_expired()) {
        return false;
    }
    
    return control_info_.holder_user_id == user_id;
}

bool ControlSyncService::session_has_control(const std::string& session_id) const {
    // 先查找会话对应的用户
    int64_t user_id = 0;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        auto it = session_user_map_.find(session_id);
        if (it == session_user_map_.end()) {
            return false;
        }
        user_id = it->second;
    }
    
    return has_control(user_id);
}

void ControlSyncService::on_control_change(ControlChangeCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callbacks_.push_back(std::move(callback));
}

void ControlSyncService::update_control(const ControlInfo& info) {
    ControlInfo old_info;
    {
        std::lock_guard<std::mutex> lock(info_mutex_);
        old_info = control_info_;
        control_info_ = info;
    }
    
    // 检查是否有变化
    if (old_info.is_held != info.is_held ||
        old_info.holder_user_id != info.holder_user_id) {
        notify_change(old_info, info);
    }
}

void ControlSyncService::associate_session(const std::string& session_id, 
                                            int64_t user_id) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    session_user_map_[session_id] = user_id;
    
    std::cout << "[ControlSync] Associated session " << session_id 
              << " to user " << user_id << std::endl;
}

void ControlSyncService::disassociate_session(const std::string& session_id) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    session_user_map_.erase(session_id);
}

void ControlSyncService::sync_loop() {
    while (running_) {
        fetch_control_status();
        
        // 等待下一次同步
        for (int i = 0; i < config_.sync_interval_ms / 100 && running_; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

bool ControlSyncService::fetch_control_status() {
    // TODO: 实现真正的 HTTP 请求
    // GET {control_plane_url}/api/v1/control/status
    // 
    // 响应格式:
    // {
    //     "control_held": true,
    //     "holder_user_id": 1,
    //     "holder_username": "admin",
    //     "holder_session_id": "xxx",
    //     "acquired_at": "2024-01-19T10:00:00Z",
    //     "expires_at": "2024-01-19T10:05:00Z"
    // }
    
    // 模拟实现：实际项目中需要使用 HTTP 库
    // 可选方案：
    // 1. libcurl / cpr
    // 2. Boost.Beast HTTP 客户端
    // 3. 通过 Unix Socket 与 Control Plane 通信
    // 4. 通过 Redis Pub/Sub 订阅控制权变更
    
    return true;
}

void ControlSyncService::notify_change(const ControlInfo& old_info, 
                                        const ControlInfo& new_info) {
    std::vector<ControlChangeCallback> callbacks_copy;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callbacks_copy = callbacks_;
    }
    
    for (const auto& cb : callbacks_copy) {
        try {
            cb(old_info, new_info);
        } catch (const std::exception& e) {
            std::cerr << "[ControlSync] Callback error: " << e.what() << std::endl;
        }
    }
    
    std::cout << "[ControlSync] Control changed: "
              << (new_info.is_held ? new_info.holder_username : "none")
              << std::endl;
}

} // namespace qyh::dataplane
