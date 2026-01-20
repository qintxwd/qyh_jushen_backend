/**
 * @file webrtc_peer.cpp
 * @brief WebRTC Peer 实现
 */

#include "media_plane/webrtc_peer.hpp"
#include "media_plane/config.hpp"

#include <iostream>

namespace qyh::mediaplane {

WebRTCPeer::WebRTCPeer(const std::string& peer_id, const Config& config)
    : peer_id_(peer_id)
    , config_(config)
{
}

WebRTCPeer::~WebRTCPeer() {
    stop();
}

bool WebRTCPeer::init(GstElement* pipeline) {
    main_pipeline_ = pipeline;
    return create_peer_bin();
}

bool WebRTCPeer::create_peer_bin() {
    // 创建 Bin
    peer_bin_ = gst_bin_new(("peer_bin_" + peer_id_).c_str());
    if (!peer_bin_) {
        std::cerr << "Failed to create peer bin" << std::endl;
        return false;
    }

    // 输入队列：解耦 Source/Network 线程
    GstElement* queue = gst_element_factory_make("queue", nullptr);
    if (!queue) {
        std::cerr << "Failed to create queue" << std::endl;
        return false;
    }
    g_object_set(queue,
                 "max-size-buffers", 1,
                 "leaky", 2,  // downstream
                 nullptr);

    // 创建 webrtcbin 元素
    webrtcbin_ = gst_element_factory_make("webrtcbin", "webrtcbin");
    if (!webrtcbin_) {
        std::cerr << "Failed to create webrtcbin element" << std::endl;
        return false;
    }

    g_object_set(webrtcbin_,
                 "bundle-policy", 3,  // GST_WEBRTC_BUNDLE_POLICY_MAX_BUNDLE
                 nullptr);

    GstElement* convert = nullptr;
    GstElement* caps_filter = nullptr;
    GstElement* encoder = nullptr;
    GstElement* payloader = nullptr;

    const std::string codec = config_.encoding.codec;

    if (codec == "h264") {
        if (config_.jetson.use_nvenc) {
            // Jetson 硬件编码路径
            // 上游已经输出 NV12(NVMM)，无需重复 nvvidconv/capsfilter
            encoder = gst_element_factory_make("nvv4l2h264enc", nullptr);
            if (encoder) {
                g_object_set(encoder,
                             "bitrate", static_cast<guint>(config_.encoding.bitrate * 1000),
                             "control-rate", 1,
                             "preset-level", 1,
                             "iframeinterval", config_.encoding.keyframe_interval,
                             "insert-sps-pps", TRUE,
                             nullptr);
            }
        } else {
            // CPU 软编码路径
            convert = gst_element_factory_make("videoconvert", nullptr);
            encoder = gst_element_factory_make("x264enc", nullptr);
            if (encoder) {
                g_object_set(encoder,
                             "bitrate", config_.encoding.bitrate,
                             "tune", 0x04,
                             "speed-preset", 1,
                             "key-int-max", config_.encoding.keyframe_interval,
                             nullptr);
            }
        }

        payloader = gst_element_factory_make("rtph264pay", nullptr);
        if (payloader) {
            g_object_set(payloader, "config-interval", 1, "pt", 96, nullptr);
        }
    } else if (codec == "vp8") {
        // VP8 仅提供软件编码兜底
        convert = gst_element_factory_make("videoconvert", nullptr);
        encoder = gst_element_factory_make("vp8enc", nullptr);
        if (encoder) {
            g_object_set(encoder,
                         "target-bitrate", static_cast<guint>(config_.encoding.bitrate * 1000),
                         "deadline", 1,
                         "keyframe-max-dist", config_.encoding.keyframe_interval,
                         nullptr);
        }

        payloader = gst_element_factory_make("rtpvp8pay", nullptr);
        if (payloader) {
            g_object_set(payloader, "pt", 96, nullptr);
        }
    } else if (codec == "h265" || codec == "hevc") {
        if (config_.jetson.use_nvenc) {
            // Jetson 硬件编码路径
            encoder = gst_element_factory_make("nvv4l2h265enc", nullptr);
            if (encoder) {
                g_object_set(encoder,
                             "bitrate", static_cast<guint>(config_.encoding.bitrate * 1000),
                             "control-rate", 1,
                             "preset-level", 1,
                             "iframeinterval", config_.encoding.keyframe_interval,
                             "insert-sps-pps", TRUE,
                             nullptr);
            }
        } else {
            // 非 Jetson 环境暂不提供 H.265 软件编码
            std::cerr << "H.265 software encoding not supported" << std::endl;
        }

        payloader = gst_element_factory_make("rtph265pay", nullptr);
        if (payloader) {
            g_object_set(payloader, "config-interval", 1, "pt", 96, nullptr);
        }
    }

    if (!encoder || !payloader) {
        std::cerr << "Failed to create encoder or payloader" << std::endl;
        return false;
    }

    gst_bin_add(GST_BIN(peer_bin_), queue);
    if (convert) {
        gst_bin_add(GST_BIN(peer_bin_), convert);
    }
    if (caps_filter) {
        gst_bin_add(GST_BIN(peer_bin_), caps_filter);
    }
    gst_bin_add_many(GST_BIN(peer_bin_), encoder, payloader, webrtcbin_, nullptr);

    bool link_ok = false;
    if (convert) {
        link_ok = gst_element_link_many(queue, convert, encoder, payloader, nullptr);
    } else {
        link_ok = gst_element_link_many(queue, encoder, payloader, nullptr);
    }

    if (!link_ok) {
        std::cerr << "Failed to link peer elements" << std::endl;
        return false;
    }

    GstPad* src_pad = gst_element_get_static_pad(payloader, "src");
    GstPad* sink_pad = gst_element_request_pad_simple(webrtcbin_, "sink_%u");
    if (!src_pad || !sink_pad || gst_pad_link(src_pad, sink_pad) != GST_PAD_LINK_OK) {
        std::cerr << "Failed to link to webrtcbin" << std::endl;
        if (src_pad) gst_object_unref(src_pad);
        if (sink_pad) gst_object_unref(sink_pad);
        return false;
    }
    gst_object_unref(src_pad);
    gst_object_unref(sink_pad);

    GstPad* queue_sink = gst_element_get_static_pad(queue, "sink");
    sink_pad_ = gst_ghost_pad_new("sink", queue_sink);
    gst_element_add_pad(peer_bin_, sink_pad_);
    gst_object_unref(queue_sink);

    // 配置 STUN/TURN
    configure_ice_servers();

    // 连接信号
    g_signal_connect(webrtcbin_, "on-negotiation-needed",
                     G_CALLBACK(on_negotiation_needed), this);
    g_signal_connect(webrtcbin_, "on-ice-candidate",
                     G_CALLBACK(on_ice_candidate), this);
    g_signal_connect(webrtcbin_, "notify::ice-gathering-state",
                     G_CALLBACK(on_ice_gathering_state_changed), this);
    g_signal_connect(webrtcbin_, "notify::ice-connection-state",
                     G_CALLBACK(on_ice_connection_state_changed), this);

    return true;
}

void WebRTCPeer::configure_ice_servers() {
    // 配置 STUN 服务器
    for (const auto& stun : config_.webrtc.stun_servers) {
        g_object_set(webrtcbin_, "stun-server", stun.c_str(), nullptr);
        break;  // webrtcbin 只支持一个 STUN
    }
    
    // TODO: 配置 TURN 服务器
}

void WebRTCPeer::create_offer() {
    GstPromise* promise = gst_promise_new_with_change_func(
        on_offer_created, this, nullptr);
    
    g_signal_emit_by_name(webrtcbin_, "create-offer", nullptr, promise);
}

void WebRTCPeer::create_answer() {
    GstPromise* promise = gst_promise_new_with_change_func(
        on_answer_created, this, nullptr);
    
    g_signal_emit_by_name(webrtcbin_, "create-answer", nullptr, promise);
}

void WebRTCPeer::set_remote_description(const std::string& sdp, 
                                         const std::string& type) {
    GstSDPMessage* sdp_msg;
    gst_sdp_message_new(&sdp_msg);
    gst_sdp_message_parse_buffer(
        reinterpret_cast<const guint8*>(sdp.c_str()), 
        static_cast<guint>(sdp.size()), 
        sdp_msg);
    
    GstWebRTCSDPType sdp_type = (type == "offer") 
        ? GST_WEBRTC_SDP_TYPE_OFFER 
        : GST_WEBRTC_SDP_TYPE_ANSWER;
    
    GstWebRTCSessionDescription* desc = 
        gst_webrtc_session_description_new(sdp_type, sdp_msg);
    
    GstPromise* promise = gst_promise_new();
    g_signal_emit_by_name(webrtcbin_, "set-remote-description", desc, promise);
    
    gst_promise_wait(promise);
    gst_promise_unref(promise);
    gst_webrtc_session_description_free(desc);
}

void WebRTCPeer::add_ice_candidate(const std::string& candidate,
                                    const std::string& sdp_mid,
                                    int sdp_mline_index) {
    g_signal_emit_by_name(webrtcbin_, "add-ice-candidate", 
                          static_cast<guint>(sdp_mline_index),
                          candidate.c_str());
    (void)sdp_mid;  // webrtcbin 使用 mline_index
}

void WebRTCPeer::start() {
    if (peer_bin_ && main_pipeline_) {
        // 注意: Bin 添加到 Pipeline 后，Pipeline 会尝试同步状态
        // 但我们可能需要显式控制
        gst_element_sync_state_with_parent(peer_bin_);
        state_ = PeerState::CONNECTING;
    }
}

void WebRTCPeer::stop() {
    if (peer_bin_) {
        // 断开连接时，先从管道中移除
        if (main_pipeline_) {
            gst_element_set_state(peer_bin_, GST_STATE_NULL);
            gst_bin_remove(GST_BIN(main_pipeline_), peer_bin_);
            gst_object_unref(peer_bin_);
        } else {
            gst_element_set_state(peer_bin_, GST_STATE_NULL);
            gst_object_unref(peer_bin_);
        }
        
        peer_bin_ = nullptr;
        webrtcbin_ = nullptr;
        sink_pad_ = nullptr;
    }
    state_ = PeerState::DISCONNECTED;
}

// ==================== 静态回调 ====================

void WebRTCPeer::on_negotiation_needed(GstElement* /*webrtc*/, gpointer user_data) {
    auto* self = static_cast<WebRTCPeer*>(user_data);
    std::cout << "Negotiation needed for peer: " << self->peer_id_ << std::endl;
}

void WebRTCPeer::on_ice_candidate(GstElement* /*webrtc*/, 
                                   guint mline_index,
                                   gchar* candidate, 
                                   gpointer user_data) {
    auto* self = static_cast<WebRTCPeer*>(user_data);
    
    if (self->ice_callback_) {
        self->ice_callback_(candidate, "", static_cast<int>(mline_index));
    }
}

void WebRTCPeer::on_ice_gathering_state_changed(GstElement* webrtc,
                                                 GParamSpec* /*pspec*/,
                                                 gpointer user_data) {
    auto* self = static_cast<WebRTCPeer*>(user_data);
    
    GstWebRTCICEGatheringState state;
    g_object_get(webrtc, "ice-gathering-state", &state, nullptr);
    
    const char* state_str = "unknown";
    switch (state) {
        case GST_WEBRTC_ICE_GATHERING_STATE_NEW: state_str = "new"; break;
        case GST_WEBRTC_ICE_GATHERING_STATE_GATHERING: state_str = "gathering"; break;
        case GST_WEBRTC_ICE_GATHERING_STATE_COMPLETE: state_str = "complete"; break;
    }
    
    std::cout << "ICE gathering state: " << state_str 
              << " for peer: " << self->peer_id_ << std::endl;
}

void WebRTCPeer::on_ice_connection_state_changed(GstElement* webrtc,
                                                  GParamSpec* /*pspec*/,
                                                  gpointer user_data) {
    auto* self = static_cast<WebRTCPeer*>(user_data);
    
    GstWebRTCICEConnectionState state;
    g_object_get(webrtc, "ice-connection-state", &state, nullptr);
    
    switch (state) {
        case GST_WEBRTC_ICE_CONNECTION_STATE_NEW:
        case GST_WEBRTC_ICE_CONNECTION_STATE_CHECKING:
            self->state_ = PeerState::CONNECTING;
            break;
        case GST_WEBRTC_ICE_CONNECTION_STATE_CONNECTED:
        case GST_WEBRTC_ICE_CONNECTION_STATE_COMPLETED:
            self->state_ = PeerState::CONNECTED;
            break;
        case GST_WEBRTC_ICE_CONNECTION_STATE_DISCONNECTED:
        case GST_WEBRTC_ICE_CONNECTION_STATE_CLOSED:
            self->state_ = PeerState::DISCONNECTED;
            break;
        case GST_WEBRTC_ICE_CONNECTION_STATE_FAILED:
            self->state_ = PeerState::FAILED;
            break;
    }
    
    if (self->state_callback_) {
        self->state_callback_(self->state_);
    }
}

void WebRTCPeer::on_offer_created(GstPromise* promise, gpointer user_data) {
    auto* self = static_cast<WebRTCPeer*>(user_data);
    
    const GstStructure* reply = gst_promise_get_reply(promise);
    GstWebRTCSessionDescription* offer = nullptr;
    gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, nullptr);
    
    if (offer) {
        // 设置本地描述
        GstPromise* local_promise = gst_promise_new();
        g_signal_emit_by_name(self->webrtcbin_, "set-local-description", offer, local_promise);
        gst_promise_wait(local_promise);
        gst_promise_unref(local_promise);
        
        // 转换为字符串
        gchar* sdp_str = gst_sdp_message_as_text(offer->sdp);
        
        if (self->sdp_callback_) {
            self->sdp_callback_(sdp_str, "offer");
        }
        
        g_free(sdp_str);
        gst_webrtc_session_description_free(offer);
    }
    
    gst_promise_unref(promise);
}

void WebRTCPeer::on_answer_created(GstPromise* promise, gpointer user_data) {
    auto* self = static_cast<WebRTCPeer*>(user_data);
    
    const GstStructure* reply = gst_promise_get_reply(promise);
    GstWebRTCSessionDescription* answer = nullptr;
    gst_structure_get(reply, "answer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &answer, nullptr);
    
    if (answer) {
        // 设置本地描述
        GstPromise* local_promise = gst_promise_new();
        g_signal_emit_by_name(self->webrtcbin_, "set-local-description", answer, local_promise);
        gst_promise_wait(local_promise);
        gst_promise_unref(local_promise);
        
        // 转换为字符串
        gchar* sdp_str = gst_sdp_message_as_text(answer->sdp);
        
        if (self->sdp_callback_) {
            self->sdp_callback_(sdp_str, "answer");
        }
        
        g_free(sdp_str);
        gst_webrtc_session_description_free(answer);
    }
    
    gst_promise_unref(promise);
}

} // namespace qyh::mediaplane
