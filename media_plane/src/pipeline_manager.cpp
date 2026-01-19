/**
 * @file pipeline_manager.cpp
 * @brief GStreamer 管道管理器实现
 */

#include "media_plane/pipeline_manager.hpp"
#include "media_plane/config.hpp"
#include "media_plane/webrtc_peer.hpp"
#include "media_plane/signaling_server.hpp"

#include <iostream>
#include <algorithm>
#include <cstdlib>

namespace qyh::mediaplane {

PipelineManager::PipelineManager(const Config& config)
    : config_(config)
{
    // 复制视频源配置（优先使用 video.sources）
    const auto& sources = config.video.sources.empty() ? 
                          config.video_sources : config.video.sources;
    
    for (const auto& src : sources) {
        VideoSource vs;
        vs.name = src.name;
        vs.device = src.device;
        vs.topic = src.topic;  // ROS2 话题
        vs.type = src.type;
        vs.enabled = src.enabled;
        video_sources_.push_back(vs);
    }
    
    // 确保有测试源可用
    bool has_test = std::any_of(video_sources_.begin(), video_sources_.end(),
                                 [](const VideoSource& vs) { return vs.type == "test"; });
    if (!has_test) {
        VideoSource test_src;
        test_src.name = "test_pattern";
        test_src.type = "test";
        test_src.enabled = true;
        video_sources_.push_back(test_src);
    }
}

PipelineManager::~PipelineManager() {
    stop_sources();
    
    if (main_loop_) {
        g_main_loop_quit(main_loop_);
    }
    
    if (gst_thread_.joinable()) {
        gst_thread_.join();
    }
    
    if (main_loop_) {
        g_main_loop_unref(main_loop_);
    }
}

bool PipelineManager::init() {
    // 创建 GMainLoop
    main_loop_ = g_main_loop_new(nullptr, FALSE);

#ifdef ENABLE_ROS2
    if (config_.ros2.enabled) {
#if defined(_WIN32)
        if (config_.ros2.domain_id >= 0) {
            _putenv_s("ROS_DOMAIN_ID", std::to_string(config_.ros2.domain_id).c_str());
        }
#else
        if (config_.ros2.domain_id >= 0) {
            setenv("ROS_DOMAIN_ID", std::to_string(config_.ros2.domain_id).c_str(), 1);
        }
#endif
        ROS2ImageSourceFactory::instance().init_ros2();
    }
#endif
    
    std::cout << "Pipeline manager initialized with " 
              << video_sources_.size() << " video sources" << std::endl;
    return true;
}

bool PipelineManager::start_sources() {
    for (auto& source : video_sources_) {
        if (!source.enabled) {
            continue;
        }
        
        if (!create_source_pipeline(source)) {
            std::cerr << "Failed to create source pipeline for: " << source.name << std::endl;
            continue;
        }
        
        std::cout << "Prepared video source: " << source.name 
                  << " (type: " << source.type << ")" << std::endl;
    }
    
    // 启动 GStreamer 线程
    gst_thread_ = std::thread([this]() {
        std::cout << "GStreamer main loop started" << std::endl;
        g_main_loop_run(main_loop_);
        std::cout << "GStreamer main loop stopped" << std::endl;
    });
    
    return true;
}

void PipelineManager::stop_sources() {
    // 停止所有 peer
    {
        std::lock_guard<std::mutex> lock(peers_mutex_);
        for (auto& [id, peer] : peers_) {
            peer->stop();
        }
        peers_.clear();
    }
    
#ifdef ENABLE_ROS2
    // 停止所有 ROS2 图像源
    {
        std::lock_guard<std::mutex> lock(ros2_sources_mutex_);
        for (auto& [topic, ros2_src] : ros2_sources_) {
            ros2_src->stop();
        }
        ros2_sources_.clear();
    }
#endif
    
    // 停止视频源
    for (auto& source : video_sources_) {
        if (source.element) {
            gst_element_set_state(source.element, GST_STATE_NULL);
            gst_object_unref(source.element);
            source.element = nullptr;
        }
    }
    
    if (main_pipeline_) {
        gst_element_set_state(main_pipeline_, GST_STATE_NULL);
        gst_object_unref(main_pipeline_);
        main_pipeline_ = nullptr;
    }
}

bool PipelineManager::create_source_pipeline(VideoSource& source) {
    GstElement* src = nullptr;
    
    if (source.type == "v4l2") {
        src = gst_element_factory_make("v4l2src", source.name.c_str());
        if (src && !source.device.empty()) {
            g_object_set(src, "device", source.device.c_str(), nullptr);
        }
    } else if (source.type == "nvarguscamerasrc" || source.type == "nvargus") {
        // Jetson CSI 摄像头
        src = gst_element_factory_make("nvarguscamerasrc", source.name.c_str());
    } else if (source.type == "videotestsrc" || source.type == "test") {
        // 测试源
        src = gst_element_factory_make("videotestsrc", source.name.c_str());
        if (src) {
            g_object_set(src, "is-live", TRUE, nullptr);
        }
    }
    
    if (!src) {
        std::cerr << "Failed to create source element for type: " << source.type << std::endl;
        return false;
    }
    
    // 创建 tee 用于多路输出
    GstElement* tee = gst_element_factory_make("tee", 
        (source.name + "_tee").c_str());
    
    if (!tee) {
        gst_object_unref(src);
        return false;
    }
    
    source.element = src;
    source.tee = tee;
    
    return true;
}

GstElement* PipelineManager::create_video_source_element(const VideoSource* source) {
    GstElement* video_src = nullptr;
    
    if (!source) {
        // 无视频源配置，使用测试源
        video_src = gst_element_factory_make("videotestsrc", nullptr);
        if (video_src) {
            g_object_set(video_src, "is-live", TRUE, "pattern", 0, nullptr);
        }
        return video_src;
    }
    
#ifdef ENABLE_ROS2
    if (source->type == "ros2") {
        // ROS2 图像源 - 返回 appsrc 元素
        // 查找或创建 ROS2ImageSource
        std::lock_guard<std::mutex> lock(ros2_sources_mutex_);
        
        auto it = ros2_sources_.find(source->topic);
        if (it != ros2_sources_.end()) {
            // 已有该话题的源，直接返回 appsrc
            return it->second->get_appsrc();
        }
        
        // 创建新的 ROS2ImageSource
        ROS2ImageSourceConfig ros2_config;
        ros2_config.topic_name = source->topic;
        ros2_config.queue_size = 1;
        
        auto& factory = ROS2ImageSourceFactory::instance();
        auto ros2_source = factory.create_source(ros2_config);
        
        if (ros2_source) {
            ros2_source->start();
            ros2_sources_[source->topic] = ros2_source;
            
            std::cout << "Created ROS2 image source for topic: " << source->topic << std::endl;
            return ros2_source->get_appsrc();
        } else {
            std::cerr << "Failed to create ROS2 image source for topic: " << source->topic << std::endl;
            // 回退到测试源
            video_src = gst_element_factory_make("videotestsrc", nullptr);
            if (video_src) {
                g_object_set(video_src, "is-live", TRUE, "pattern", 0, nullptr);
            }
            return video_src;
        }
    }
#endif
    
    if (source->type == "v4l2") {
        video_src = gst_element_factory_make("v4l2src", nullptr);
        if (video_src && !source->device.empty()) {
            g_object_set(video_src, "device", source->device.c_str(), nullptr);
        }
    } else if (source->type == "nvarguscamerasrc" || source->type == "nvargus") {
        video_src = gst_element_factory_make("nvarguscamerasrc", nullptr);
    } else if (source->type == "videotestsrc" || source->type == "test") {
        video_src = gst_element_factory_make("videotestsrc", nullptr);
        if (video_src) {
            g_object_set(video_src, "is-live", TRUE, "pattern", 0, nullptr);
        }
    }
    
    return video_src;
}

GstElement* PipelineManager::create_encoder() {
    GstElement* encoder = nullptr;
    
    if (config_.jetson.use_nvenc) {
        // Jetson 硬件编码
        if (config_.encoding.codec == "h264") {
            encoder = gst_element_factory_make("nvv4l2h264enc", nullptr);
        } else if (config_.encoding.codec == "h265") {
            encoder = gst_element_factory_make("nvv4l2h265enc", nullptr);
        }
        
        if (encoder) {
            g_object_set(encoder,
                         "bitrate", config_.encoding.bitrate * 1000,
                         nullptr);
        }
    }
    
    if (!encoder) {
        // 软件编码后备
        if (config_.encoding.codec == "h264") {
            encoder = gst_element_factory_make("x264enc", nullptr);
            if (encoder) {
                g_object_set(encoder,
                             "bitrate", config_.encoding.bitrate,
                             "tune", 0x04,  // zerolatency
                             "speed-preset", 1,  // ultrafast
                             "key-int-max", config_.encoding.keyframe_interval,
                             nullptr);
            }
        } else if (config_.encoding.codec == "vp8") {
            encoder = gst_element_factory_make("vp8enc", nullptr);
            if (encoder) {
                g_object_set(encoder,
                             "target-bitrate", config_.encoding.bitrate * 1000,
                             nullptr);
            }
        }
    }
    
    return encoder;
}

std::shared_ptr<WebRTCPeer> PipelineManager::create_peer(
    const std::string& peer_id,
    const std::string& video_source) {
    
    // 查找视频源
    VideoSource* source = nullptr;
    for (auto& vs : video_sources_) {
        if (vs.name == video_source && vs.enabled) {
            source = &vs;
            break;
        }
    }
    
    if (!source && !video_sources_.empty()) {
        // 使用第一个可用源
        for (auto& vs : video_sources_) {
            if (vs.enabled) {
                source = &vs;
                break;
            }
        }
    }
    
    // 创建 peer
    auto peer = std::make_shared<WebRTCPeer>(peer_id, config_);
    if (!peer->init()) {
        std::cerr << "Failed to init WebRTC peer" << std::endl;
        report_error(peer_id, "Failed to initialize WebRTC peer");
        return nullptr;
    }
    
    // 创建视频源元素
    GstElement* video_src = create_video_source_element(source);
    
    if (!video_src) {
        std::cerr << "Failed to create video source element" << std::endl;
        report_error(peer_id, "Failed to create video source");
        return nullptr;
    }
    
    std::string source_name = source ? source->name : "test_pattern";
    
    // 添加视频源到 peer
    if (!peer->add_video_source(source_name, video_src)) {
        std::cerr << "Failed to add video source to peer" << std::endl;
        gst_object_unref(video_src);
        report_error(peer_id, "Failed to add video source to peer");
        return nullptr;
    }
    
    // 存储 peer
    {
        std::lock_guard<std::mutex> lock(peers_mutex_);
        peers_[peer_id] = peer;
        peer_sources_[peer_id] = source_name;
    }
    
    // 更新统计
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.active_peers++;
        stats_.total_peers_created++;
    }
    
    // 启动 peer
    peer->start();
    
    std::cout << "Created WebRTC peer: " << peer_id 
              << " with source: " << source_name << std::endl;
    
    return peer;
}

void PipelineManager::remove_peer(const std::string& peer_id) {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    
    auto it = peers_.find(peer_id);
    if (it != peers_.end()) {
        it->second->stop();
        peers_.erase(it);
        peer_sources_.erase(peer_id);
        
        // 更新统计
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            if (stats_.active_peers > 0) {
                stats_.active_peers--;
            }
        }
        
        std::cout << "Removed WebRTC peer: " << peer_id << std::endl;
    }
}

bool PipelineManager::switch_peer_source(const std::string& peer_id, 
                                          const std::string& new_source) {
    // 检查新源是否可用
    if (!is_source_available(new_source)) {
        report_error(peer_id, "Source not available: " + new_source);
        return false;
    }
    
    std::lock_guard<std::mutex> lock(peers_mutex_);
    
    auto it = peers_.find(peer_id);
    if (it == peers_.end()) {
        return false;
    }
    
    auto current_source = peer_sources_[peer_id];
    if (current_source == new_source) {
        // 已经是这个源了
        return true;
    }
    
    // 目前实现：停止旧 peer，需要客户端重新请求
    // TODO: 实现无缝切换（需要动态重新连接管道）
    it->second->stop();
    peers_.erase(it);
    peer_sources_.erase(peer_id);
    
    std::cout << "Peer " << peer_id << " source switch requested from " 
              << current_source << " to " << new_source 
              << " (requires reconnection)" << std::endl;
    
    return false;  // 返回 false 表示需要重新连接
}

std::vector<std::string> PipelineManager::get_available_sources() const {
    std::vector<std::string> sources;
    for (const auto& vs : video_sources_) {
        if (vs.enabled) {
            sources.push_back(vs.name);
        }
    }
    return sources;
}

bool PipelineManager::is_source_available(const std::string& source_name) const {
    return std::any_of(video_sources_.begin(), video_sources_.end(),
                       [&source_name](const VideoSource& vs) {
                           return vs.name == source_name && vs.enabled;
                       });
}

size_t PipelineManager::peer_count() const {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    return peers_.size();
}

PipelineStats PipelineManager::get_stats() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

void PipelineManager::report_error(const std::string& peer_id, const std::string& error) {
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.error_count++;
    }
    
    if (error_callback_) {
        error_callback_(peer_id, error);
    }
    
    std::cerr << "Pipeline error for peer " << peer_id << ": " << error << std::endl;
}

gboolean PipelineManager::bus_callback(GstBus* /*bus*/, GstMessage* msg, gpointer data) {
    auto* self = static_cast<PipelineManager*>(data);
    
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError* err;
            gchar* debug;
            gst_message_parse_error(msg, &err, &debug);
            std::cerr << "GStreamer error: " << err->message << std::endl;
            g_error_free(err);
            g_free(debug);
            break;
        }
        case GST_MESSAGE_WARNING: {
            GError* err;
            gchar* debug;
            gst_message_parse_warning(msg, &err, &debug);
            std::cerr << "GStreamer warning: " << err->message << std::endl;
            g_error_free(err);
            g_free(debug);
            break;
        }
        case GST_MESSAGE_EOS:
            std::cout << "End of stream" << std::endl;
            break;
        default:
            break;
    }
    
    (void)self;
    return TRUE;
}

} // namespace qyh::mediaplane
