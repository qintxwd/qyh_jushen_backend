/**
 * @file rate_limiter.cpp
 * @brief 状态推送频率限制器实现
 */

#include "data_plane/rate_limiter.hpp"
#include <algorithm>
#include <numeric>

namespace qyh::dataplane {

// ==================== TopicRateLimiter ====================

TopicRateLimiter::TopicRateLimiter(int max_rate_hz)
    : max_rate_hz_(max_rate_hz)
    , last_publish_(std::chrono::steady_clock::now())
    , stats_start_(std::chrono::steady_clock::now())
{
    set_max_rate(max_rate_hz);
}

bool TopicRateLimiter::should_publish() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        now - last_publish_);
    
    if (elapsed >= min_interval_) {
        last_publish_ = now;
        ++publish_count_;
        return true;
    }
    return false;
}

void TopicRateLimiter::set_max_rate(int hz) {
    max_rate_hz_ = std::max(1, hz);
    min_interval_ = std::chrono::microseconds(1000000 / max_rate_hz_);
}

double TopicRateLimiter::actual_rate() const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        now - stats_start_).count();
    
    if (elapsed <= 0) {
        return 0.0;
    }
    return static_cast<double>(publish_count_) / elapsed;
}

void TopicRateLimiter::reset() {
    publish_count_ = 0;
    stats_start_ = std::chrono::steady_clock::now();
    last_publish_ = stats_start_;
}

// ==================== DeltaDetector ====================

bool DeltaDetector::has_changed(const std::string& topic, 
                                 const std::vector<uint8_t>& data,
                                 size_t threshold) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 计算新数据的哈希
    size_t new_hash = compute_hash(data);
    
    auto it = hash_cache_.find(topic);
    if (it == hash_cache_.end()) {
        // 首次数据，认为有变化
        hash_cache_[topic] = new_hash;
        cache_[topic] = data;
        return true;
    }
    
    // 快速哈希比较
    if (it->second != new_hash) {
        hash_cache_[topic] = new_hash;
        cache_[topic] = data;
        return true;
    }
    
    // 哈希相同，进行详细比较（处理哈希碰撞）
    const auto& cached = cache_[topic];
    if (cached.size() != data.size()) {
        cache_[topic] = data;
        return true;
    }
    
    // 如果设置了阈值，检查差异字节数
    if (threshold > 0) {
        size_t diff_count = 0;
        for (size_t i = 0; i < data.size(); ++i) {
            if (cached[i] != data[i]) {
                ++diff_count;
                if (diff_count > threshold) {
                    cache_[topic] = data;
                    return true;
                }
            }
        }
        return false;
    }
    
    // 精确比较
    if (cached != data) {
        cache_[topic] = data;
        return true;
    }
    
    return false;
}

void DeltaDetector::clear(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_.erase(topic);
    hash_cache_.erase(topic);
}

void DeltaDetector::clear_all() {
    std::lock_guard<std::mutex> lock(mutex_);
    cache_.clear();
    hash_cache_.clear();
}

size_t DeltaDetector::compute_hash(const std::vector<uint8_t>& data) {
    // FNV-1a hash
    size_t hash = 14695981039346656037ULL;
    for (uint8_t byte : data) {
        hash ^= byte;
        hash *= 1099511628211ULL;
    }
    return hash;
}

// ==================== StateAggregator ====================

StateAggregator::StateAggregator() = default;

void StateAggregator::configure_topic(const TopicConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    topic_configs_[config.name] = config;
    
    if (rate_limiters_.find(config.name) != rate_limiters_.end()) {
        rate_limiters_[config.name].set_max_rate(config.max_rate_hz);
    }
}

bool StateAggregator::should_publish(const std::string& topic, 
                                      const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查话题是否启用
    auto config_it = topic_configs_.find(topic);
    if (config_it != topic_configs_.end() && !config_it->second.enabled) {
        return false;
    }
    
    // 检查差量
    if (config_it != topic_configs_.end() && config_it->second.delta_only) {
        if (!delta_detector_.has_changed(topic, data)) {
            return false;
        }
    }
    
    // 检查频率限制
    auto& limiter = get_limiter(topic);
    if (!limiter.should_publish()) {
        drop_counts_[topic]++;
        return false;
    }
    
    return true;
}

void StateAggregator::force_publish(const std::string& topic) {
    std::lock_guard<std::mutex> lock(mutex_);
    delta_detector_.clear(topic);
}

std::vector<StateAggregator::TopicStats> StateAggregator::get_stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<TopicStats> stats;
    for (const auto& [topic, limiter] : rate_limiters_) {
        TopicStats s;
        s.topic = topic;
        s.actual_rate = limiter.actual_rate();
        s.total_published = static_cast<int>(s.actual_rate); // 简化
        
        auto drop_it = drop_counts_.find(topic);
        s.total_dropped = (drop_it != drop_counts_.end()) ? drop_it->second : 0;
        
        stats.push_back(s);
    }
    return stats;
}

void StateAggregator::reset_stats() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [topic, limiter] : rate_limiters_) {
        limiter.reset();
    }
    drop_counts_.clear();
    delta_detector_.clear_all();
}

TopicRateLimiter& StateAggregator::get_limiter(const std::string& topic) {
    auto it = rate_limiters_.find(topic);
    if (it != rate_limiters_.end()) {
        return it->second;
    }
    
    // 创建新的限制器
    int rate = default_rate_hz_;
    auto config_it = topic_configs_.find(topic);
    if (config_it != topic_configs_.end()) {
        rate = config_it->second.max_rate_hz;
    }
    
    rate_limiters_.emplace(topic, rate);
    return rate_limiters_.at(topic);
}

} // namespace qyh::dataplane
