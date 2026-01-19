/**
 * @file metrics.hpp
 * @brief 监控指标收集器
 * 
 * 收集性能指标，支持 Prometheus 格式导出
 */

#pragma once

#include <string>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <sstream>

namespace qyh::dataplane {

/**
 * @brief 计数器指标
 */
class Counter {
public:
    explicit Counter(const std::string& name, const std::string& help = "")
        : name_(name), help_(help) {}
    
    void inc(double value = 1.0) { value_ += value; }
    double get() const { return value_.load(); }
    void reset() { value_ = 0; }
    
    const std::string& name() const { return name_; }
    const std::string& help() const { return help_; }
    
private:
    std::string name_;
    std::string help_;
    std::atomic<double> value_{0};
};

/**
 * @brief 仪表盘指标（可增可减）
 */
class Gauge {
public:
    explicit Gauge(const std::string& name, const std::string& help = "")
        : name_(name), help_(help) {}
    
    void set(double value) { value_ = value; }
    void inc(double value = 1.0) { value_ += value; }
    void dec(double value = 1.0) { value_ -= value; }
    double get() const { return value_.load(); }
    
    const std::string& name() const { return name_; }
    const std::string& help() const { return help_; }
    
private:
    std::string name_;
    std::string help_;
    std::atomic<double> value_{0};
};

/**
 * @brief 直方图指标
 */
class Histogram {
public:
    explicit Histogram(const std::string& name, 
                       const std::string& help = "",
                       const std::vector<double>& buckets = 
                           {0.001, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0, 2.5, 5.0, 10.0})
        : name_(name), help_(help), buckets_(buckets)
    {
        bucket_counts_.resize(buckets_.size() + 1, 0);
    }
    
    void observe(double value) {
        std::lock_guard<std::mutex> lock(mutex_);
        sum_ += value;
        ++count_;
        
        for (size_t i = 0; i < buckets_.size(); ++i) {
            if (value <= buckets_[i]) {
                ++bucket_counts_[i];
                return;
            }
        }
        ++bucket_counts_.back(); // +Inf bucket
    }
    
    double sum() const { return sum_; }
    uint64_t count() const { return count_; }
    
    const std::string& name() const { return name_; }
    const std::string& help() const { return help_; }
    const std::vector<double>& buckets() const { return buckets_; }
    std::vector<uint64_t> bucket_counts() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return bucket_counts_;
    }
    
private:
    std::string name_;
    std::string help_;
    std::vector<double> buckets_;
    std::vector<uint64_t> bucket_counts_;
    double sum_ = 0;
    uint64_t count_ = 0;
    mutable std::mutex mutex_;
};

/**
 * @brief 监控指标收集器
 */
class MetricsCollector {
public:
    /**
     * @brief 获取单例实例
     */
    static MetricsCollector& instance() {
        static MetricsCollector instance;
        return instance;
    }
    
    // ==================== 预定义指标 ====================
    
    // 连接指标
    Gauge connections_total{"dataplane_connections_total", 
                            "Total number of active WebSocket connections"};
    Counter connections_accepted{"dataplane_connections_accepted_total",
                                  "Total number of accepted connections"};
    Counter connections_rejected{"dataplane_connections_rejected_total",
                                  "Total number of rejected connections"};
    Counter connections_closed{"dataplane_connections_closed_total",
                                "Total number of closed connections"};
    
    // 消息指标
    Counter messages_received{"dataplane_messages_received_total",
                               "Total number of received messages"};
    Counter messages_sent{"dataplane_messages_sent_total",
                           "Total number of sent messages"};
    Counter bytes_received{"dataplane_bytes_received_total",
                            "Total bytes received"};
    Counter bytes_sent{"dataplane_bytes_sent_total",
                        "Total bytes sent"};
    
    // 延迟指标
    Histogram message_latency{"dataplane_message_latency_seconds",
                               "Message processing latency in seconds"};
    Histogram publish_latency{"dataplane_publish_latency_seconds",
                               "State publish latency in seconds"};
    
    // ROS2 指标
    Counter ros2_messages_received{"dataplane_ros2_messages_received_total",
                                    "Total ROS2 messages received"};
    Counter ros2_messages_published{"dataplane_ros2_messages_published_total",
                                     "Total ROS2 messages published"};
    Gauge ros2_connected{"dataplane_ros2_connected",
                          "Whether ROS2 is connected (1) or not (0)"};
    
    // Watchdog 指标
    Counter watchdog_triggers{"dataplane_watchdog_triggers_total",
                               "Total number of watchdog triggers"};
    Gauge watchdog_active_sessions{"dataplane_watchdog_active_sessions",
                                    "Number of sessions with active heartbeat"};
    
    // 错误指标
    Counter errors_total{"dataplane_errors_total",
                          "Total number of errors"};
    Counter auth_failures{"dataplane_auth_failures_total",
                           "Total authentication failures"};
    
    // ==================== 方法 ====================
    
    /**
     * @brief 导出为 Prometheus 格式
     */
    std::string export_prometheus() const {
        std::ostringstream oss;
        
        // 辅助函数：输出指标
        auto write_counter = [&](const Counter& c) {
            oss << "# HELP " << c.name() << " " << c.help() << "\n";
            oss << "# TYPE " << c.name() << " counter\n";
            oss << c.name() << " " << c.get() << "\n";
        };
        
        auto write_gauge = [&](const Gauge& g) {
            oss << "# HELP " << g.name() << " " << g.help() << "\n";
            oss << "# TYPE " << g.name() << " gauge\n";
            oss << g.name() << " " << g.get() << "\n";
        };
        
        auto write_histogram = [&](const Histogram& h) {
            oss << "# HELP " << h.name() << " " << h.help() << "\n";
            oss << "# TYPE " << h.name() << " histogram\n";
            
            auto counts = h.bucket_counts();
            const auto& buckets = h.buckets();
            uint64_t cumulative = 0;
            
            for (size_t i = 0; i < buckets.size(); ++i) {
                cumulative += counts[i];
                oss << h.name() << "_bucket{le=\"" << buckets[i] << "\"} " 
                    << cumulative << "\n";
            }
            cumulative += counts.back();
            oss << h.name() << "_bucket{le=\"+Inf\"} " << cumulative << "\n";
            oss << h.name() << "_sum " << h.sum() << "\n";
            oss << h.name() << "_count " << h.count() << "\n";
        };
        
        // 输出所有指标
        write_gauge(connections_total);
        write_counter(connections_accepted);
        write_counter(connections_rejected);
        write_counter(connections_closed);
        
        write_counter(messages_received);
        write_counter(messages_sent);
        write_counter(bytes_received);
        write_counter(bytes_sent);
        
        write_histogram(message_latency);
        write_histogram(publish_latency);
        
        write_counter(ros2_messages_received);
        write_counter(ros2_messages_published);
        write_gauge(ros2_connected);
        
        write_counter(watchdog_triggers);
        write_gauge(watchdog_active_sessions);
        
        write_counter(errors_total);
        write_counter(auth_failures);
        
        // 添加自定义指标
        {
            std::lock_guard<std::mutex> lock(custom_mutex_);
            for (const auto& [name, value] : custom_gauges_) {
                oss << "# TYPE " << name << " gauge\n";
                oss << name << " " << value << "\n";
            }
        }
        
        return oss.str();
    }
    
    /**
     * @brief 导出为 JSON 格式
     */
    std::string export_json() const {
        std::ostringstream oss;
        oss << "{\n";
        oss << "  \"connections\": {\n";
        oss << "    \"total\": " << connections_total.get() << ",\n";
        oss << "    \"accepted\": " << connections_accepted.get() << ",\n";
        oss << "    \"rejected\": " << connections_rejected.get() << ",\n";
        oss << "    \"closed\": " << connections_closed.get() << "\n";
        oss << "  },\n";
        oss << "  \"messages\": {\n";
        oss << "    \"received\": " << messages_received.get() << ",\n";
        oss << "    \"sent\": " << messages_sent.get() << ",\n";
        oss << "    \"bytes_received\": " << bytes_received.get() << ",\n";
        oss << "    \"bytes_sent\": " << bytes_sent.get() << "\n";
        oss << "  },\n";
        oss << "  \"ros2\": {\n";
        oss << "    \"connected\": " << (ros2_connected.get() > 0 ? "true" : "false") << ",\n";
        oss << "    \"messages_received\": " << ros2_messages_received.get() << ",\n";
        oss << "    \"messages_published\": " << ros2_messages_published.get() << "\n";
        oss << "  },\n";
        oss << "  \"watchdog\": {\n";
        oss << "    \"triggers\": " << watchdog_triggers.get() << ",\n";
        oss << "    \"active_sessions\": " << watchdog_active_sessions.get() << "\n";
        oss << "  },\n";
        oss << "  \"errors\": {\n";
        oss << "    \"total\": " << errors_total.get() << ",\n";
        oss << "    \"auth_failures\": " << auth_failures.get() << "\n";
        oss << "  }\n";
        oss << "}\n";
        return oss.str();
    }
    
    /**
     * @brief 设置自定义 Gauge 值
     */
    void set_custom_gauge(const std::string& name, double value) {
        std::lock_guard<std::mutex> lock(custom_mutex_);
        custom_gauges_[name] = value;
    }
    
    /**
     * @brief 重置所有指标
     */
    void reset_all() {
        connections_total.set(0);
        connections_accepted.reset();
        connections_rejected.reset();
        connections_closed.reset();
        messages_received.reset();
        messages_sent.reset();
        bytes_received.reset();
        bytes_sent.reset();
        ros2_messages_received.reset();
        ros2_messages_published.reset();
        ros2_connected.set(0);
        watchdog_triggers.reset();
        watchdog_active_sessions.set(0);
        errors_total.reset();
        auth_failures.reset();
        
        std::lock_guard<std::mutex> lock(custom_mutex_);
        custom_gauges_.clear();
    }
    
private:
    MetricsCollector() = default;
    
    std::unordered_map<std::string, double> custom_gauges_;
    mutable std::mutex custom_mutex_;
};

/**
 * @brief 延迟计时器（RAII 风格）
 */
class LatencyTimer {
public:
    explicit LatencyTimer(Histogram& histogram)
        : histogram_(histogram)
        , start_(std::chrono::steady_clock::now())
    {}
    
    ~LatencyTimer() {
        auto end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration<double>(end - start_).count();
        histogram_.observe(duration);
    }
    
    // 禁止拷贝
    LatencyTimer(const LatencyTimer&) = delete;
    LatencyTimer& operator=(const LatencyTimer&) = delete;
    
private:
    Histogram& histogram_;
    std::chrono::steady_clock::time_point start_;
};

// 便捷宏
#define METRICS MetricsCollector::instance()
#define MEASURE_LATENCY(histogram) LatencyTimer _timer##__LINE__(histogram)

} // namespace qyh::dataplane
