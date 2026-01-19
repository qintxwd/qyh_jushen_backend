/**
 * @file pipeline_manager.cpp
 * @brief GStreamer 管道管理器实现
 */

#include "media_plane/pipeline_manager.hpp"
#include "media_plane/config.hpp"
#include "media_plane/webrtc_peer.hpp"
#include "media_plane/signaling_server.hpp"

#include <iostream>

namespace qyh::mediaplane {

PipelineManager::PipelineManager(const Config& config)
    : config_(config)
{
    // 复制视频源配置
    for (const auto& src : config.video_sources) {
        VideoSource vs;
        vs.name = src.name;
        vs.device = src.device;
        vs.type = src.type;
        vs.enabled = src.enabled;
        video_sources_.push_back(vs);
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
    
    std::cout << "Pipeline manager initialized" << std::endl;
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
        
        std::cout << "Started video source: " << source.name << std::endl;
    }
    
    // 启动 GStreamer 线程
    gst_thread_ = std::thread([this]() {
        g_main_loop_run(main_loop_);
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
        if (src) {
            g_object_set(src, "device", source.device.c_str(), nullptr);
        }
    } else if (source.type == "nvarguscamerasrc") {
        // Jetson CSI 摄像头
        src = gst_element_factory_make("nvarguscamerasrc", source.name.c_str());
    } else if (source.type == "videotestsrc") {
        // 测试源
        src = gst_element_factory_make("videotestsrc", source.name.c_str());
        g_object_set(src, "is-live", TRUE, nullptr);
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
        return nullptr;
    }
    
    // TODO: 连接视频源到 peer
    // 这需要更复杂的管道管理，使用 tee 分流
    
    // 存储 peer
    {
        std::lock_guard<std::mutex> lock(peers_mutex_);
        peers_[peer_id] = peer;
    }
    
    // 启动 peer
    peer->start();
    
    std::cout << "Created WebRTC peer: " << peer_id 
              << " with source: " << (source ? source->name : "none") << std::endl;
    
    return peer;
}

void PipelineManager::remove_peer(const std::string& peer_id) {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    
    auto it = peers_.find(peer_id);
    if (it != peers_.end()) {
        it->second->stop();
        peers_.erase(it);
        std::cout << "Removed WebRTC peer: " << peer_id << std::endl;
    }
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
