/**
 * @file vr_session.hpp
 * @brief VR 专用会话管理
 * 
 * VR 遥操作使用专用通道，单连接限制：
 * - 同一时间只允许一个 VR 客户端连接
 * - 天然互斥，无需复杂的控制权管理
 * - 断开时自动通知 Control Plane
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <atomic>
#include <chrono>
#include <functional>

namespace qyh::dataplane {

// 前向声明
class Session;

/**
 * @brief VR 客户端信息
 */
struct VRClientInfo {
    std::string device;           // 设备型号 (如 "PICO 4")
    std::string version;          // 客户端版本
    std::string session_id;       // WebSocket 会话 ID
    std::chrono::system_clock::time_point connected_at;
};

/**
 * @brief VR 连接状态变化回调
 */
using VRConnectionCallback = std::function<void(bool connected, const VRClientInfo& info)>;

/**
 * @brief VR 会话管理器（单例）
 * 
 * 管理 VR 专用通道的单连接限制
 */
class VRSessionManager {
public:
    /**
     * @brief 获取单例实例
     */
    static VRSessionManager& instance();
    
    // 禁止拷贝和移动
    VRSessionManager(const VRSessionManager&) = delete;
    VRSessionManager& operator=(const VRSessionManager&) = delete;
    
    /**
     * @brief 尝试接受 VR 连接
     * 
     * @param session WebSocket 会话
     * @param device 设备型号
     * @param version 客户端版本
     * @return true 允许连接; false 拒绝（已有 VR 连接）
     */
    bool try_accept(std::shared_ptr<Session> session,
                    const std::string& device = "PICO 4",
                    const std::string& version = "1.0.0");
    
    /**
     * @brief VR 断开连接
     * 
     * @param session_id 会话 ID
     * @param reason 断开原因: "client_close", "heartbeat_timeout", "error"
     */
    void on_disconnect(const std::string& session_id, const std::string& reason);
    
    /**
     * @brief 检查是否有 VR 连接
     */
    bool is_connected() const;
    
    /**
     * @brief 获取当前 VR 客户端信息
     * 
     * @return 客户端信息，如果未连接则返回空信息
     */
    VRClientInfo get_client_info() const;
    
    /**
     * @brief 获取当前 VR 会话
     * 
     * @return VR 会话指针，如果未连接则返回 nullptr
     */
    std::shared_ptr<Session> get_session() const;
    
    /**
     * @brief 检查会话是否是 VR 会话
     * 
     * @param session_id 会话 ID
     */
    bool is_vr_session(const std::string& session_id) const;
    
    /**
     * @brief 设置连接状态变化回调
     * 
     * 用于通知 Control Plane VR 连接/断开
     */
    void set_connection_callback(VRConnectionCallback callback);
    
    /**
     * @brief 设置 Control Plane URL
     * 
     * @param url Control Plane 的 URL (如 "http://localhost:8000")
     */
    void set_control_plane_url(const std::string& url);

    /**
     * @brief 设置 Control Plane 内部鉴权 Token
     */
    void set_internal_token(const std::string& token);
    
private:
    VRSessionManager() = default;
    
    /**
     * @brief 通知 Control Plane VR 连接状态变化
     */
    void notify_control_plane(bool connected, const VRClientInfo& info,
                               const std::string& reason = "");
    
private:
    mutable std::mutex mutex_;
    
    // 当前 VR 会话（单连接）
    std::weak_ptr<Session> vr_session_;
    VRClientInfo client_info_;
    std::atomic<bool> connected_{false};
    
    // 回调
    VRConnectionCallback connection_callback_;
    
    // Control Plane URL
    std::string control_plane_url_ = "http://127.0.0.1:8000";
    std::string internal_token_;
};

/**
 * @brief VR 会话 RAII 守卫
 * 
 * 用于在 Session 析构时自动清理 VR 连接状态
 */
class VRSessionGuard {
public:
    VRSessionGuard(const std::string& session_id)
        : session_id_(session_id)
        , active_(VRSessionManager::instance().is_vr_session(session_id))
    {}
    
    ~VRSessionGuard() {
        if (active_) {
            VRSessionManager::instance().on_disconnect(session_id_, "session_destroyed");
        }
    }
    
    // 禁止拷贝，允许移动
    VRSessionGuard(const VRSessionGuard&) = delete;
    VRSessionGuard& operator=(const VRSessionGuard&) = delete;
    
    VRSessionGuard(VRSessionGuard&& other) noexcept
        : session_id_(std::move(other.session_id_))
        , active_(other.active_)
    {
        other.active_ = false;
    }
    
private:
    std::string session_id_;
    bool active_;
};

} // namespace qyh::dataplane
