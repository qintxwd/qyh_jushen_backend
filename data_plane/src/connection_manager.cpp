/**
 * @file connection_manager.cpp
 * @brief 连接管理器实现
 */

#include "data_plane/connection_manager.hpp"
#include <algorithm>
#include <iostream>

namespace qyh::dataplane {

ConnectionManager::ConnectionManager(const ConnectionManagerConfig& config)
    : config_(config)
{
}

ConnectionManager::AcceptResult ConnectionManager::try_accept(
    const std::string& remote_address,
    const std::string& client_type) {
    
    AcceptResult result;
    
    // 检查黑名单
    if (is_blacklisted(remote_address)) {
        result.accepted = false;
        result.reject_reason = RejectReason::BLACKLISTED;
        result.message = "IP is blacklisted";
        ++total_rejected_;
        return result;
    }
    
    // 检查连接频率
    if (!check_rate_limit(remote_address)) {
        result.accepted = false;
        result.reject_reason = RejectReason::RATE_LIMITED;
        result.message = "Connection rate limit exceeded";
        ++total_rejected_;
        return result;
    }
    
    // 检查单 IP 连接数
    if (connections_from_ip(remote_address) >= config_.max_connections_per_ip) {
        result.accepted = false;
        result.reject_reason = RejectReason::MAX_PER_IP;
        result.message = "Too many connections from this IP";
        ++total_rejected_;
        return result;
    }
    
    // 检查总连接数
    int current = connection_count_.load();
    if (current >= config_.max_connections) {
        // 尝试驱逐低优先级连接
        if (config_.enable_priority_eviction) {
            // VR 客户端有更高优先级
            ConnectionPriority incoming_priority = ConnectionPriority::LOW;
            if (client_type == "vr") {
                incoming_priority = ConnectionPriority::CRITICAL;
            } else if (client_type == "mobile") {
                incoming_priority = ConnectionPriority::NORMAL;
            }
            
            std::string evicted = try_evict_low_priority(incoming_priority);
            if (!evicted.empty()) {
                result.accepted = true;
                result.message = "Accepted after evicting lower priority connection";
                ++total_accepted_;
                return result;
            }
        }
        
        result.accepted = false;
        result.reject_reason = RejectReason::MAX_CONNECTIONS;
        result.message = "Maximum connections reached";
        ++total_rejected_;
        return result;
    }
    
    result.accepted = true;
    ++total_accepted_;
    return result;
}

void ConnectionManager::register_connection(const std::string& session_id,
                                             const ConnectionInfo& info) {
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        connections_[session_id] = info;
    }
    
    {
        std::lock_guard<std::mutex> lock(ip_mutex_);
        ip_counts_[info.remote_address]++;
    }
    
    ++connection_count_;
    
    std::cout << "[ConnectionManager] Registered: " << session_id 
              << " from " << info.remote_address 
              << " (" << info.client_type << ")" << std::endl;
}

void ConnectionManager::unregister_connection(const std::string& session_id) {
    std::string ip;
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        auto it = connections_.find(session_id);
        if (it != connections_.end()) {
            ip = it->second.remote_address;
            connections_.erase(it);
        }
    }
    
    if (!ip.empty()) {
        std::lock_guard<std::mutex> lock(ip_mutex_);
        auto it = ip_counts_.find(ip);
        if (it != ip_counts_.end()) {
            if (--it->second <= 0) {
                ip_counts_.erase(it);
            }
        }
    }
    
    --connection_count_;
    
    std::cout << "[ConnectionManager] Unregistered: " << session_id << std::endl;
}

void ConnectionManager::update_priority(const std::string& session_id,
                                         ConnectionPriority priority) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(session_id);
    if (it != connections_.end()) {
        it->second.priority = priority;
    }
}

void ConnectionManager::update_activity(const std::string& session_id) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(session_id);
    if (it != connections_.end()) {
        it->second.last_activity = std::chrono::steady_clock::now();
    }
}

void ConnectionManager::record_message(const std::string& session_id,
                                        bool is_sent,
                                        size_t bytes) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(session_id);
    if (it != connections_.end()) {
        if (is_sent) {
            it->second.messages_sent++;
            it->second.bytes_sent += bytes;
        } else {
            it->second.messages_received++;
            it->second.bytes_received += bytes;
        }
        it->second.last_activity = std::chrono::steady_clock::now();
    }
}

std::optional<ConnectionInfo> ConnectionManager::get_connection(
    const std::string& session_id) const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    auto it = connections_.find(session_id);
    if (it != connections_.end()) {
        return it->second;
    }
    return std::nullopt;
}

std::vector<ConnectionInfo> ConnectionManager::get_all_connections() const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    std::vector<ConnectionInfo> result;
    result.reserve(connections_.size());
    for (const auto& [id, info] : connections_) {
        result.push_back(info);
    }
    return result;
}

int ConnectionManager::connections_from_ip(const std::string& ip) const {
    std::lock_guard<std::mutex> lock(ip_mutex_);
    auto it = ip_counts_.find(ip);
    return (it != ip_counts_.end()) ? it->second : 0;
}

bool ConnectionManager::kick_connection(const std::string& session_id,
                                         const std::string& reason) {
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        if (connections_.find(session_id) == connections_.end()) {
            return false;
        }
    }
    
    // 调用回调
    KickCallback cb;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        cb = kick_callback_;
    }
    
    if (cb) {
        cb(session_id, reason);
    }
    
    ++total_kicked_;
    std::cout << "[ConnectionManager] Kicked: " << session_id 
              << " reason: " << reason << std::endl;
    
    return true;
}

int ConnectionManager::kick_idle_connections() {
    auto now = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(config_.idle_timeout_seconds);
    
    std::vector<std::string> to_kick;
    
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (const auto& [id, info] : connections_) {
            if (now - info.last_activity > timeout) {
                // 不踢出高优先级连接
                if (info.priority < ConnectionPriority::HIGH) {
                    to_kick.push_back(id);
                }
            }
        }
    }
    
    for (const auto& id : to_kick) {
        kick_connection(id, "Idle timeout");
    }
    
    return static_cast<int>(to_kick.size());
}

void ConnectionManager::blacklist_ip(const std::string& ip,
                                      std::chrono::seconds duration) {
    std::lock_guard<std::mutex> lock(blacklist_mutex_);
    blacklist_[ip] = std::chrono::steady_clock::now() + duration;
    std::cout << "[ConnectionManager] Blacklisted IP: " << ip << std::endl;
}

void ConnectionManager::unblacklist_ip(const std::string& ip) {
    std::lock_guard<std::mutex> lock(blacklist_mutex_);
    blacklist_.erase(ip);
}

bool ConnectionManager::is_blacklisted(const std::string& ip) const {
    std::lock_guard<std::mutex> lock(blacklist_mutex_);
    auto it = blacklist_.find(ip);
    if (it == blacklist_.end()) {
        return false;
    }
    return std::chrono::steady_clock::now() < it->second;
}

void ConnectionManager::set_kick_callback(KickCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    kick_callback_ = std::move(callback);
}

ConnectionManager::Stats ConnectionManager::get_stats() const {
    Stats stats;
    stats.total_connections = connection_count_.load();
    stats.total_accepted = total_accepted_.load();
    stats.total_rejected = total_rejected_.load();
    stats.total_kicked = total_kicked_.load();
    stats.current_queue_size = 0; // 暂未实现队列
    
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        for (const auto& [id, info] : connections_) {
            stats.connections_by_type[info.client_type]++;
            stats.connections_by_priority[static_cast<int>(info.priority)]++;
        }
    }
    
    return stats;
}

bool ConnectionManager::check_rate_limit(const std::string& ip) {
    auto now = std::chrono::steady_clock::now();
    auto window = std::chrono::minutes(1);
    
    std::lock_guard<std::mutex> lock(history_mutex_);
    
    auto& history = connection_history_[ip];
    
    // 清理过期记录
    history.erase(
        std::remove_if(history.begin(), history.end(),
            [&](const auto& t) { return now - t > window; }),
        history.end());
    
    // 检查是否超过限制
    if (static_cast<int>(history.size()) >= config_.connection_rate_limit_per_minute) {
        return false;
    }
    
    history.push_back(now);
    return true;
}

std::string ConnectionManager::try_evict_low_priority(ConnectionPriority min_priority) {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    
    // 找到最低优先级的连接
    std::string lowest_id;
    ConnectionPriority lowest_priority = min_priority;
    std::chrono::steady_clock::time_point oldest_activity;
    
    for (const auto& [id, info] : connections_) {
        if (info.priority < lowest_priority ||
            (info.priority == lowest_priority && 
             (lowest_id.empty() || info.last_activity < oldest_activity))) {
            lowest_id = id;
            lowest_priority = info.priority;
            oldest_activity = info.last_activity;
        }
    }
    
    if (!lowest_id.empty() && lowest_priority < min_priority) {
        // 不在锁内调用 kick，避免死锁
        std::string to_kick = lowest_id;
        return to_kick;
    }
    
    return "";
}

void ConnectionManager::cleanup_blacklist() {
    auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(blacklist_mutex_);
    
    for (auto it = blacklist_.begin(); it != blacklist_.end();) {
        if (now >= it->second) {
            it = blacklist_.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace qyh::dataplane
