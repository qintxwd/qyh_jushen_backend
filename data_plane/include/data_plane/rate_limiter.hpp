/**
 * @file rate_limiter.hpp
 * @brief 状态推送频率限制器
 * 
 * 实现按话题的频率限流，避免过度推送
 * 支持差量更新检测
 */

#pragma once

#include <chrono>
#include <unordered_map>
#include <mutex>
#include <string>
#include <vector>
#include <cstdint>
#include <functional>

namespace qyh::dataplane {

/**
 * @brief 单话题频率限制器
 */
class TopicRateLimiter {
public:
    /**
     * @brief 构造函数
     * @param max_rate_hz 最大推送频率 (Hz)
     */
    explicit TopicRateLimiter(int max_rate_hz = 30);
    
    /**
     * @brief 检查是否可以推送
     * @return 是否允许推送
     */
    bool should_publish();
    
    /**
     * @brief 设置最大频率
     * @param hz 频率 (Hz)
     */
    void set_max_rate(int hz);
    
    /**
     * @brief 获取当前实际频率
     */
    double actual_rate() const;
    
    /**
     * @brief 重置统计
     */
    void reset();
    
private:
    int max_rate_hz_;
    std::chrono::steady_clock::time_point last_publish_;
    std::chrono::microseconds min_interval_;
    
    // 统计
    int publish_count_ = 0;
    std::chrono::steady_clock::time_point stats_start_;
};

/**
 * @brief 差量检测器 - 检测数据是否有变化
 */
class DeltaDetector {
public:
    /**
     * @brief 检查数据是否有变化
     * @param topic 话题名
     * @param data 新数据
     * @param threshold 变化阈值 (字节数)
     * @return 是否有变化
     */
    bool has_changed(const std::string& topic, 
                     const std::vector<uint8_t>& data,
                     size_t threshold = 0);
    
    /**
     * @brief 清除话题缓存
     */
    void clear(const std::string& topic);
    
    /**
     * @brief 清除所有缓存
     */
    void clear_all();
    
private:
    std::unordered_map<std::string, std::vector<uint8_t>> cache_;
    std::unordered_map<std::string, size_t> hash_cache_;
    mutable std::mutex mutex_;
    
    size_t compute_hash(const std::vector<uint8_t>& data);
};

/**
 * @brief 状态聚合推送器
 * 
 * 负责按配置频率聚合和推送状态数据
 */
class StateAggregator {
public:
    /**
     * @brief 话题配置
     */
    struct TopicConfig {
        std::string name;
        int max_rate_hz = 30;
        bool delta_only = false;    // 仅推送变化的数据
        bool enabled = true;
    };
    
    /**
     * @brief 构造函数
     */
    StateAggregator();
    
    /**
     * @brief 配置话题
     */
    void configure_topic(const TopicConfig& config);
    
    /**
     * @brief 配置默认频率
     */
    void set_default_rate(int hz) { default_rate_hz_ = hz; }
    
    /**
     * @brief 检查是否应该推送
     * @param topic 话题名
     * @param data 数据
     * @return 是否应该推送
     */
    bool should_publish(const std::string& topic, 
                        const std::vector<uint8_t>& data);
    
    /**
     * @brief 强制推送（忽略频率限制）
     */
    void force_publish(const std::string& topic);
    
    /**
     * @brief 获取话题统计
     */
    struct TopicStats {
        std::string topic;
        double actual_rate;
        int total_published;
        int total_dropped;
    };
    std::vector<TopicStats> get_stats() const;
    
    /**
     * @brief 重置所有统计
     */
    void reset_stats();
    
private:
    int default_rate_hz_ = 30;
    
    std::unordered_map<std::string, TopicConfig> topic_configs_;
    std::unordered_map<std::string, TopicRateLimiter> rate_limiters_;
    std::unordered_map<std::string, int> drop_counts_;
    
    DeltaDetector delta_detector_;
    mutable std::mutex mutex_;
    
    TopicRateLimiter& get_limiter(const std::string& topic);
};

} // namespace qyh::dataplane
