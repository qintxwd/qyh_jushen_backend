/**
 * @file control_sync.hpp
 * @brief 控制权同步服务
 * 
 * 负责与 Control Plane 同步控制权状态
 * 确保 Data Plane 知道哪个用户有控制权
 */

#pragma once

#include <string>
#include <mutex>
#include <atomic>
#include <functional>
#include <chrono>
#include <thread>
#include <memory>
#include <unordered_map>
#include <vector>

namespace qyh::dataplane {

/**
 * @brief 控制权信息
 */
struct ControlInfo {
    bool is_held = false;
    int64_t holder_user_id = 0;
    std::string holder_username;
    std::string holder_session_id;
    std::chrono::steady_clock::time_point acquired_at;
    std::chrono::steady_clock::time_point expires_at;
    
    bool is_expired() const {
        return std::chrono::steady_clock::now() > expires_at;
    }
};

/**
 * @brief 控制权变更回调
 */
using ControlChangeCallback = std::function<void(const ControlInfo& old_info, 
                                                   const ControlInfo& new_info)>;

/**
 * @brief 控制权同步服务
 * 
 * 定期从 Control Plane 获取控制权状态
 * 或接收 Control Plane 的推送通知
 */
class ControlSyncService {
public:
    /**
     * @brief 配置
     */
    struct Config {
        std::string control_plane_url = "http://127.0.0.1:8000";
        std::string internal_token;
        int sync_interval_ms = 1000;        // 同步间隔
        int timeout_ms = 5000;              // 请求超时
        bool enabled = true;
    };
    
    /**
     * @brief 构造函数
     */
    explicit ControlSyncService(const Config& config);
    
    ~ControlSyncService();
    
    /**
     * @brief 启动同步服务
     */
    void start();
    
    /**
     * @brief 停止同步服务
     */
    void stop();
    
    /**
     * @brief 手动触发同步
     */
    void sync_now();
    
    /**
     * @brief 获取当前控制权信息
     */
    ControlInfo get_control_info() const;
    
    /**
     * @brief 检查用户是否有控制权
     * @param user_id 用户 ID
     * @return 是否有控制权
     */
    bool has_control(int64_t user_id) const;
    
    /**
     * @brief 检查会话是否有控制权
     * @param session_id WebSocket 会话 ID
     * @return 是否有控制权
     */
    bool session_has_control(const std::string& session_id) const;
    
    /**
     * @brief 注册控制权变更回调
     */
    void on_control_change(ControlChangeCallback callback);
    
    /**
     * @brief 从 Control Plane 推送更新控制权
     * 
     * 当 Control Plane 通过内部通道通知控制权变更时调用
     */
    void update_control(const ControlInfo& info);
    
    /**
     * @brief 关联 WebSocket 会话到用户
     * @param session_id WebSocket 会话 ID
     * @param user_id 用户 ID
     */
    void associate_session(const std::string& session_id, int64_t user_id);
    
    /**
     * @brief 解除会话关联
     */
    void disassociate_session(const std::string& session_id);
    
private:
    /**
     * @brief 同步线程函数
     */
    void sync_loop();
    
    /**
     * @brief 从 Control Plane 获取控制权状态
     */
    bool fetch_control_status();
    
    /**
     * @brief 通知控制权变更
     */
    void notify_change(const ControlInfo& old_info, const ControlInfo& new_info);
    
private:
    Config config_;
    
    ControlInfo control_info_;
    mutable std::mutex info_mutex_;
    
    // 会话到用户的映射
    std::unordered_map<std::string, int64_t> session_user_map_;
    mutable std::mutex session_mutex_;
    
    // 回调
    std::vector<ControlChangeCallback> callbacks_;
    std::mutex callback_mutex_;
    
    // 同步线程
    std::atomic<bool> running_{false};
    std::thread sync_thread_;
};

/**
 * @brief 控制权检查器（辅助类）
 * 
 * 用于快速检查控制权，可注入到 MessageHandler
 */
class ControlChecker {
public:
    explicit ControlChecker(ControlSyncService& service)
        : service_(service) {}
    
    /**
     * @brief 检查用户是否可以执行控制命令
     * @param user_id 用户 ID
     * @param permission 所需权限 ("teleop", "navigation", etc.)
     * @return 是否允许
     */
    bool can_control(int64_t user_id, const std::string& permission = "teleop") const {
        // 首先检查是否有控制权
        if (!service_.has_control(user_id)) {
            return false;
        }
        // 权限检查可以扩展
        return true;
    }
    
    /**
     * @brief 检查会话是否可以执行控制命令
     */
    bool session_can_control(const std::string& session_id,
                             const std::string& permission = "teleop") const {
        return service_.session_has_control(session_id);
    }
    
private:
    ControlSyncService& service_;
};

} // namespace qyh::dataplane
