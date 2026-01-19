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

bool WebRTCPeer::init() {
    return create_pipeline();
}

bool WebRTCPeer::create_pipeline() {
    // 创建管道
    std::string pipeline_desc = "webrtcbin name=webrtcbin bundle-policy=max-bundle";
    
    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);
    
    if (error) {
        std::cerr << "Failed to create pipeline: " << error->message << std::endl;
        g_error_free(error);
        return false;
    }
    
    // 获取 webrtcbin
    webrtcbin_ = gst_bin_get_by_name(GST_BIN(pipeline_), "webrtcbin");
    if (!webrtcbin_) {
        std::cerr << "Failed to get webrtcbin" << std::endl;
        return false;
    }
    
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

bool WebRTCPeer::add_video_source(const std::string& /*source_name*/, 
                                   GstElement* src_element) {
    if (!pipeline_ || !webrtcbin_) {
        return false;
    }
    
    // 创建编码管道
    // videosrc -> videoconvert -> encoder -> rtph264pay -> webrtcbin
    
    GstElement* convert = gst_element_factory_make("videoconvert", nullptr);
    GstElement* encoder = nullptr;
    GstElement* payloader = nullptr;
    
    // 根据配置选择编码器
    if (config_.jetson.use_nvenc) {
        // Jetson 硬件编码
        encoder = gst_element_factory_make("nvv4l2h264enc", nullptr);
        if (encoder) {
            g_object_set(encoder, 
                         "bitrate", config_.encoding.bitrate * 1000,
                         "preset-level", 1,  // UltraFast
                         nullptr);
        }
    }
    
    if (!encoder) {
        // 软件编码后备
        encoder = gst_element_factory_make("x264enc", nullptr);
        if (encoder) {
            g_object_set(encoder,
                         "bitrate", config_.encoding.bitrate,
                         "tune", 0x04,  // zerolatency
                         "speed-preset", 1,  // ultrafast
                         nullptr);
        }
    }
    
    if (!encoder) {
        std::cerr << "Failed to create encoder" << std::endl;
        return false;
    }
    
    payloader = gst_element_factory_make("rtph264pay", nullptr);
    if (!payloader) {
        std::cerr << "Failed to create payloader" << std::endl;
        return false;
    }
    
    // 添加到管道
    gst_bin_add_many(GST_BIN(pipeline_), src_element, convert, encoder, payloader, nullptr);
    
    // 连接元素
    if (!gst_element_link_many(src_element, convert, encoder, payloader, nullptr)) {
        std::cerr << "Failed to link video elements" << std::endl;
        return false;
    }
    
    // 获取 payloader 的 src pad
    GstPad* src_pad = gst_element_get_static_pad(payloader, "src");
    
    // 请求 webrtcbin 的 sink pad
    GstPad* sink_pad = gst_element_request_pad_simple(webrtcbin_, "sink_%u");
    
    // 连接到 webrtcbin
    GstPadLinkReturn ret = gst_pad_link(src_pad, sink_pad);
    
    gst_object_unref(src_pad);
    gst_object_unref(sink_pad);
    
    if (ret != GST_PAD_LINK_OK) {
        std::cerr << "Failed to link to webrtcbin" << std::endl;
        return false;
    }
    
    return true;
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
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);
        state_ = PeerState::CONNECTING;
    }
}

void WebRTCPeer::stop() {
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        webrtcbin_ = nullptr;
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
