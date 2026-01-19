/**
 * @file connection_manager.hpp
 * @brief 连接管理器
 * 
 * 负责连接数限制、排队、优雅降级
 */

#pragma once

#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>

namespace qyh::dataplane {

// 前向声明
class Session;

/**
 * @brief 连接优先级
 */
enum class ConnectionPriority {
    LOW = 0,        // 普通 Web 客户端
    NORMAL = 1,     // 认证用户
    HIGH = 2,       // 操作员
    CRITICAL = 3    // VR 控制端
};

/**
 * @brief 连接信息
 */
struct ConnectionInfo {
    std::string session_id;
    std::string client_type;        // "web", "vr", "mobile"
    std::string remote_address;
    int64_t user_id = 0;
    ConnectionPriority priority = ConnectionPriority::LOW;
    std::chrono::steady_clock::time_point connected_at;
    std::chrono::steady_clock::time_point last_activity;
    size_t messages_sent = 0;
    size_t messages_received = 0;
    size_t bytes_sent = 0;
    size_t bytes_received = 0;
};

/**
 * @brief 连接被拒绝的原因
 */
enum class RejectReason {
    NONE = 0,
    MAX_CONNECTIONS,        // 达到最大连接数
    MAX_PER_IP,             // 单 IP 连接数过多
    RATE_LIMITED,           // 连接频率过高
    BLACKLISTED,            // IP 在黑名单中
    LOW_PRIORITY_EVICTED    // 被高优先级连接挤掉
};

/**
 * @brief 连接管理器配置
 */
struct ConnectionManagerConfig {
    int max_connections = 100;
    int max_connections_per_ip = 10;
    int max_queue_size = 20;
    int connection_rate_limit_per_minute = 60;
    int idle_timeout_seconds = 300;
    bool enable_priority_eviction = true;
};

/**
 * @brief 连接管理器
 */
class ConnectionManager {
public:
    /**
     * @brief 连接请求结果
     */
    struct AcceptResult {
        bool accepted = false;
        RejectReason reject_reason = RejectReason::NONE;
        std::string message;
        int queue_position = -1;    // 如果排队，返回位置
    };
    
    /**
     * @brief 构造函数
     */
    explicit ConnectionManager(const ConnectionManagerConfig& config = {});
    
    /**
     * @brief 尝试接受新连接
     * @param remote_address 远程地址
     * @param client_type 客户端类型
     * @return 接受结果
     */
    AcceptResult try_accept(const std::string& remote_address,
                            const std::string& client_type);
    
    /**
     * @brief 注册连接
     * @param session_id 会话 ID
     * @param info 连接信息
     */
    void register_connection(const std::string& session_id,
                             const ConnectionInfo& info);
    
    /**
     * @brief 注销连接
     * @param session_id 会话 ID
     */
    void unregister_connection(const std::string& session_id);
    
    /**
     * @brief 更新连接优先级
     */
    void update_priority(const std::string& session_id,
                         ConnectionPriority priority);
    
    /**
     * @brief 更新连接活动时间
     */
    void update_activity(const std::string& session_id);
    
    /**
     * @brief 记录消息统计
     */
    void record_message(const std::string& session_id,
                        bool is_sent,
                        size_t bytes);
    
    /**
     * @brief 获取连接信息
     */
    std::optional<ConnectionInfo> get_connection(const std::string& session_id) const;
    
    /**
     * @brief 获取所有连接
     */
    std::vector<ConnectionInfo> get_all_connections() const;
    
    /**
     * @brief 获取当前连接数
     */
    int connection_count() const { return connection_count_.load(); }
    
    /**
     * @brief 获取某 IP 的连接数
     */
    int connections_from_ip(const std::string& ip) const;
    
    /**
     * @brief 踢出连接
     * @param session_id 会话 ID
     * @param reason 原因
     * @return 是否成功
     */
    bool kick_connection(const std::string& session_id, 
                         const std::string& reason);
    
    /**
     * @brief 踢出空闲连接
     * @return 踢出的连接数
     */
    int kick_idle_connections();
    
    /**
     * @brief 添加 IP 到黑名单
     */
    void blacklist_ip(const std::string& ip, 
                      std::chrono::seconds duration = std::chrono::seconds(3600));
    
    /**
     * @brief 从黑名单移除 IP
     */
    void unblacklist_ip(const std::string& ip);
    
    /**
     * @brief 检查 IP 是否在黑名单中
     */
    bool is_blacklisted(const std::string& ip) const;
    
    /**
     * @brief 设置踢出回调
     */
    using KickCallback = std::function<void(const std::string& session_id, 
                                             const std::string& reason)>;
    void set_kick_callback(KickCallback callback);
    
    /**
     * @brief 获取统计信息
     */
    struct Stats {
        int total_connections;
        int total_accepted;
        int total_rejected;
        int total_kicked;
        int current_queue_size;
        std::unordered_map<std::string, int> connections_by_type;
        std::unordered_map<int, int> connections_by_priority;
    };
    Stats get_stats() const;
    
private:
    /**
     * @brief 检查连接频率限制
     */
    bool check_rate_limit(const std::string& ip);
    
    /**
     * @brief 尝试驱逐低优先级连接
     */
    std::string try_evict_low_priority(ConnectionPriority min_priority);
    
    /**
     * @brief 清理过期黑名单
     */
    void cleanup_blacklist();
    
private:
    ConnectionManagerConfig config_;
    
    // 连接存储
    std::unordered_map<std::string, ConnectionInfo> connections_;
    mutable std::mutex connections_mutex_;
    
    // IP 计数
    std::unordered_map<std::string, int> ip_counts_;
    mutable std::mutex ip_mutex_;
    
    // 连接频率追踪
    std::unordered_map<std::string, std::vector<std::chrono::steady_clock::time_point>> 
        connection_history_;
    std::mutex history_mutex_;
    
    // 黑名单
    std::unordered_map<std::string, std::chrono::steady_clock::time_point> blacklist_;
    mutable std::mutex blacklist_mutex_;
    
    // 统计
    std::atomic<int> connection_count_{0};
    std::atomic<int> total_accepted_{0};
    std::atomic<int> total_rejected_{0};
    std::atomic<int> total_kicked_{0};
    
    // 回调
    KickCallback kick_callback_;
    std::mutex callback_mutex_;
};

} // namespace qyh::dataplane
