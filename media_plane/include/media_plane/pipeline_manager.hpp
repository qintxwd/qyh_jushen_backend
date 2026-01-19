/**
 * @file pipeline_manager.hpp
 * @brief GStreamer 管道管理器
 * 
 * 管理视频源和 WebRTC Peer 的生命周期
 */

#pragma once

#include <gst/gst.h>

#include <memory>
#include <unordered_map>
#include <mutex>
#include <string>
#include <vector>
#include <functional>
#include <atomic>

namespace qyh::mediaplane {

// 前向声明
class Config;
class WebRTCPeer;
class SignalingServer;

/**
 * @brief 视频源信息
 */
struct VideoSource {
    std::string name;
    std::string device;
    std::string type;  // v4l2, nvarguscamerasrc, test
    bool enabled;
    
    GstElement* element = nullptr;
    GstElement* tee = nullptr;  // 用于多路输出
};

/**
 * @brief Pipeline 统计信息
 */
struct PipelineStats {
    size_t active_peers = 0;
    size_t total_peers_created = 0;
    size_t error_count = 0;
    uint64_t bytes_sent = 0;
};

/**
 * @brief 错误回调类型
 */
using ErrorCallback = std::function<void(const std::string& peer_id, const std::string& error)>;

/**
 * @brief GStreamer 管道管理器
 */
class PipelineManager {
public:
    /**
     * @brief 构造函数
     * @param config 配置
     */
    explicit PipelineManager(const Config& config);
    
    ~PipelineManager();
    
    /**
     * @brief 初始化 GStreamer
     * @return 是否成功
     */
    bool init();
    
    /**
     * @brief 启动所有视频源
     */
    bool start_sources();
    
    /**
     * @brief 停止所有视频源
     */
    void stop_sources();
    
    /**
     * @brief 设置信令服务器
     */
    void set_signaling_server(SignalingServer* server) { signaling_server_ = server; }
    
    /**
     * @brief 设置错误回调
     */
    void set_error_callback(ErrorCallback callback) { error_callback_ = std::move(callback); }
    
    /**
     * @brief 创建新的 WebRTC Peer
     * @param peer_id Peer 标识符
     * @param video_source 请求的视频源名称
     * @return WebRTC Peer
     */
    std::shared_ptr<WebRTCPeer> create_peer(const std::string& peer_id,
                                             const std::string& video_source);
    
    /**
     * @brief 移除 WebRTC Peer
     */
    void remove_peer(const std::string& peer_id);
    
    /**
     * @brief 切换 Peer 的视频源
     * @param peer_id Peer 标识符
     * @param new_source 新的视频源名称
     * @return 是否成功
     */
    bool switch_peer_source(const std::string& peer_id, const std::string& new_source);
    
    /**
     * @brief 获取可用的视频源列表
     */
    std::vector<std::string> get_available_sources() const;
    
    /**
     * @brief 检查视频源是否可用
     */
    bool is_source_available(const std::string& source_name) const;
    
    /**
     * @brief 获取 Peer 数量
     */
    size_t peer_count() const;
    
    /**
     * @brief 获取统计信息
     */
    PipelineStats get_stats() const;
    
private:
    /**
     * @brief 创建视频源管道
     */
    bool create_source_pipeline(VideoSource& source);
    
    /**
     * @brief 创建编码器元素
     */
    GstElement* create_encoder();
    
    /**
     * @brief 创建视频源元素
     */
    GstElement* create_video_source_element(const VideoSource* source);
    
    /**
     * @brief 处理 GStreamer 总线消息
     */
    static gboolean bus_callback(GstBus* bus, GstMessage* msg, gpointer data);
    
    /**
     * @brief 报告错误
     */
    void report_error(const std::string& peer_id, const std::string& error);
    
private:
    const Config& config_;
    SignalingServer* signaling_server_ = nullptr;
    ErrorCallback error_callback_;
    
    std::vector<VideoSource> video_sources_;
    GstElement* main_pipeline_ = nullptr;
    
    std::unordered_map<std::string, std::shared_ptr<WebRTCPeer>> peers_;
    std::unordered_map<std::string, std::string> peer_sources_;  // peer_id -> source_name
    mutable std::mutex peers_mutex_;
    
    GMainLoop* main_loop_ = nullptr;
    std::thread gst_thread_;
    
    // 统计
    mutable std::mutex stats_mutex_;
    PipelineStats stats_;
};

} // namespace qyh::mediaplane
