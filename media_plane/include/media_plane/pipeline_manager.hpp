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
    std::string type;  // v4l2, nvarguscamerasrc, etc.
    bool enabled;
    
    GstElement* element = nullptr;
    GstElement* tee = nullptr;  // 用于多路输出
};

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
     * @brief 获取可用的视频源列表
     */
    std::vector<std::string> get_available_sources() const;
    
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
     * @brief 处理 GStreamer 总线消息
     */
    static gboolean bus_callback(GstBus* bus, GstMessage* msg, gpointer data);
    
private:
    const Config& config_;
    SignalingServer* signaling_server_ = nullptr;
    
    std::vector<VideoSource> video_sources_;
    GstElement* main_pipeline_ = nullptr;
    
    std::unordered_map<std::string, std::shared_ptr<WebRTCPeer>> peers_;
    mutable std::mutex peers_mutex_;
    
    GMainLoop* main_loop_ = nullptr;
    std::thread gst_thread_;
};

} // namespace qyh::mediaplane
