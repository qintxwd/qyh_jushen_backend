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

    // 创建主管道
    main_pipeline_ = gst_pipeline_new("media-server-pipeline");
    if (!main_pipeline_) {
        std::cerr << "Failed to create main pipeline" << std::endl;
        return false;
    }

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
    
    // 设置总线监视
    GstBus* bus = gst_element_get_bus(main_pipeline_);
    gst_bus_add_watch(bus, bus_callback, this);
    gst_object_unref(bus);
    
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
    
    // 启动主管道
    GstStateChangeReturn ret = gst_element_set_state(main_pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
         std::cerr << "Failed to set pipeline to PLAYING" << std::endl;
         return false;
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
    
    // 停止管道
    if (main_pipeline_) {
        gst_element_set_state(main_pipeline_, GST_STATE_NULL);
        gst_object_unref(main_pipeline_);
        main_pipeline_ = nullptr;
    }

    // 清空源指针 (不需要 unref，因为它们被 main_pipeline 拥有并释放了)
    for (auto& source : video_sources_) {
        source.element = nullptr;
        source.tee = nullptr;
    }
}

bool PipelineManager::create_source_pipeline(VideoSource& source) {
    GstElement* src = nullptr;
    
    // 1. 创建源元素
#ifdef ENABLE_ROS2
    if (source.type == "ros2") {
        std::lock_guard<std::mutex> lock(ros2_sources_mutex_);
        
        // 每个 Topic 只创建一个 ROS2 Source 对象
        if (ros2_sources_.find(source.topic) == ros2_sources_.end()) {
            ROS2ImageSourceConfig ros2_config;
            ros2_config.topic_name = source.topic;
            ros2_config.queue_size = 1;
            
            auto& factory = ROS2ImageSourceFactory::instance();
            auto ros2_source = factory.create_source(ros2_config);
            if (ros2_source) {
                ros2_source->start();
                ros2_sources_[source.topic] = ros2_source;
            }
        }
        
        if (ros2_sources_.count(source.topic)) {
            src = ros2_sources_[source.topic]->get_appsrc();
        }
    }
#endif

    if (!src) {
        if (source.type == "v4l2") {
            src = gst_element_factory_make("v4l2src", source.name.c_str());
            if (src && !source.device.empty()) {
                g_object_set(src, "device", source.device.c_str(), nullptr);
            }
        } else if (source.type == "nvarguscamerasrc" || source.type == "nvargus") {
            src = gst_element_factory_make("nvarguscamerasrc", source.name.c_str());
        } else {
            // 默认测试源
            src = gst_element_factory_make("videotestsrc", source.name.c_str());
            if (src) {
                g_object_set(src, "is-live", TRUE, nullptr);
            }
        }
    }
    
    if (!src) {
        std::cerr << "Failed to create source element for: " << source.name << std::endl;
        return false;
    }

    // 2. 创建 Tee
    GstElement* tee = gst_element_factory_make("tee", (source.name + "_tee").c_str());
    if (!tee) {
        if (g_object_is_floating(src)) gst_object_unref(src);
        return false;
    }
    g_object_set(tee, "allow-not-linked", TRUE, nullptr);

    // 3. 将元素添加到主管道 (重要：所有权转移)
    gst_bin_add(GST_BIN(main_pipeline_), src);
    gst_bin_add(GST_BIN(main_pipeline_), tee);

    // 4. 连接 Source -> [Convert] -> Tee
    GstElement* sw_convert = nullptr;
    GstElement* hw_convert = nullptr;
    GstElement* caps_filter = nullptr;

    if (config_.jetson.use_nvenc) {
        // 先用软件 videoconvert 处理 BGR/RGB 输入，避免 nvvidconv 协商失败
        sw_convert = gst_element_factory_make("videoconvert", nullptr);
        hw_convert = gst_element_factory_make("nvvidconv", nullptr);
    } else {
        sw_convert = gst_element_factory_make("videoconvert", nullptr);
    }

    if (config_.jetson.use_nvenc && sw_convert && hw_convert) {
        gst_bin_add_many(GST_BIN(main_pipeline_), sw_convert, hw_convert, nullptr);

        // 强制 nvvidconv 输出 NV12 到 Tee
        caps_filter = gst_element_factory_make("capsfilter", nullptr);
        if (caps_filter) {
            GstCaps* caps = gst_caps_from_string("video/x-raw(memory:NVMM), format=(string)NV12");
            g_object_set(caps_filter, "caps", caps, nullptr);
            gst_caps_unref(caps);
            gst_bin_add(GST_BIN(main_pipeline_), caps_filter);

            if (!gst_element_link_many(src, sw_convert, hw_convert, caps_filter, tee, nullptr)) {
                std::cerr << "Failed to link source chain with nvvidconv" << std::endl;
                return false;
            }
        } else {
            if (!gst_element_link_many(src, sw_convert, hw_convert, tee, nullptr)) {
                std::cerr << "Failed to link source chain with nvvidconv" << std::endl;
                return false;
            }
        }
    } else if (sw_convert) {
        gst_bin_add(GST_BIN(main_pipeline_), sw_convert);
        if (!gst_element_link_many(src, sw_convert, tee, nullptr)) {
            std::cerr << "Failed to link source chain" << std::endl;
            return false;
        }
    } else {
        gst_element_link(src, tee);
    }
    
    source.element = src;
    source.tee = tee;
    
    return true;
}

std::shared_ptr<WebRTCPeer> PipelineManager::create_peer(
    const std::string& peer_id,
    const std::string& video_source) {
    
    // 1. 查找源
    VideoSource* source = nullptr;
    for (auto& vs : video_sources_) {
        if (vs.name == video_source && vs.enabled) {
            source = &vs;
            break;
        }
    }
    
    if (!source && !video_sources_.empty()) {
        for (auto& vs : video_sources_) {
            if (vs.enabled) {
                source = &vs;
                break;
            }
        }
    }
    
    if (!source || !source->tee) {
        std::cerr << "Source not found or invalid: " << video_source << std::endl;
        report_error(peer_id, "Video source not available");
        return nullptr;
    }
    
    // 2. 创建 Peer
    auto peer = std::make_shared<WebRTCPeer>(peer_id, config_);
    if (!peer->init(main_pipeline_)) {
        std::cerr << "Failed to init WebRTC peer" << std::endl;
        report_error(peer_id, "Failed to initialize WebRTC peer");
        return nullptr;
    }
    
    GstElement* peer_bin = peer->get_element();
    
    // 3. 添加 Peer Bin 到主管道
    if (!gst_bin_add(GST_BIN(main_pipeline_), peer_bin)) {
        std::cerr << "Failed to add peer bin" << std::endl;
        return nullptr;
    }
    gst_element_sync_state_with_parent(peer_bin);

    // 4. 连接 Source Tee -> Peer Bin
    GstPad* tee_pad = gst_element_request_pad_simple(source->tee, "src_%u");
    GstPad* sink_pad = peer->get_sink_pad();

    if (tee_pad && sink_pad) {
        if (gst_pad_link(tee_pad, sink_pad) != GST_PAD_LINK_OK) {
            std::cerr << "Failed to link tee to peer bin" << std::endl;
            gst_element_release_request_pad(source->tee, tee_pad);
            gst_object_unref(tee_pad);
        } else {
            std::lock_guard<std::mutex> lock(peers_mutex_);
            peer_tee_pads_[peer_id] = tee_pad;
        }
    } else {
        std::cerr << "Failed to get pads for linking" << std::endl;
        if (tee_pad) {
            gst_element_release_request_pad(source->tee, tee_pad);
            gst_object_unref(tee_pad);
        }
    }
    
    // 存储 peer
    {
        std::lock_guard<std::mutex> lock(peers_mutex_);
        peers_[peer_id] = peer;
        peer_sources_[peer_id] = source->name;
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
              << " with source: " << source->name << std::endl;
    
    return peer;
}

void PipelineManager::remove_peer(const std::string& peer_id) {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    
    auto it = peers_.find(peer_id);
    if (it != peers_.end()) {
        it->second->stop();

        // 释放 Tee Request Pad
        auto pad_it = peer_tee_pads_.find(peer_id);
        if (pad_it != peer_tee_pads_.end()) {
            GstPad* pad = pad_it->second;
            GstElement* tee = gst_pad_get_parent_element(pad);
            if (tee) {
                gst_element_release_request_pad(tee, pad);
                gst_object_unref(tee);
            }
            gst_object_unref(pad);
            peer_tee_pads_.erase(pad_it);
        }

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
