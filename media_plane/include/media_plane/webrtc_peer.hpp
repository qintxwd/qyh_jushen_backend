/**
 * @file webrtc_peer.hpp
 * @brief WebRTC Peer 管理
 * 
 * 使用 GStreamer webrtcbin 实现 WebRTC 连接
 */

#pragma once

#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <gst/sdp/sdp.h>

#include <memory>
#include <string>
#include <functional>
#include <atomic>

namespace qyh::mediaplane {

// 前向声明
class Config;
class SignalingServer;

/**
 * @brief WebRTC Peer 状态
 */
enum class PeerState {
    IDLE,
    CONNECTING,
    CONNECTED,
    DISCONNECTED,
    FAILED
};

/**
 * @brief WebRTC Peer
 * 
 * 封装 GStreamer webrtcbin，管理单个 WebRTC 连接
 */
class WebRTCPeer : public std::enable_shared_from_this<WebRTCPeer> {
public:
    using SDPCallback = std::function<void(const std::string& sdp, const std::string& type)>;
    using ICECallback = std::function<void(const std::string& candidate, 
                                            const std::string& sdp_mid, 
                                            int sdp_mline_index)>;
    using StateCallback = std::function<void(PeerState state)>;
    
    /**
     * @brief 构造函数
     * @param peer_id Peer 标识符
     * @param config 配置
     */
    WebRTCPeer(const std::string& peer_id, const Config& config);
    
    ~WebRTCPeer();
    
    /**
     * @brief 初始化 WebRTC Bin
     * @param pipeline 主管道指针
     * @return 是否成功
     */
    bool init(GstElement* pipeline);
    
    /**
     * @brief 获取 Peer 的 Bin 元素
     */
    GstElement* get_element() const { return peer_bin_; }

    /**
     * @brief 获取 Peer 的 Sink Pad（用于连接源）
     */
    GstPad* get_sink_pad() const { return sink_pad_; }
    
    /**
     * @brief 创建 Offer
     */
    void create_offer();
    
    /**
     * @brief 创建 Answer
     */
    void create_answer();
    
    /**
     * @brief 设置远程 SDP
     * @param sdp SDP 字符串
     * @param type "offer" 或 "answer"
     */
    void set_remote_description(const std::string& sdp, const std::string& type);
    
    /**
     * @brief 添加 ICE Candidate
     */
    void add_ice_candidate(const std::string& candidate, 
                           const std::string& sdp_mid, 
                           int sdp_mline_index);
    
    /**
     * @brief 启动流传输
     */
    void start();
    
    /**
     * @brief 停止流传输
     */
    void stop();
    
    /**
     * @brief 获取 Peer ID
     */
    const std::string& peer_id() const { return peer_id_; }
    
    /**
     * @brief 获取状态
     */
    PeerState state() const { return state_; }
    
    // 回调设置
    void set_sdp_callback(SDPCallback cb) { sdp_callback_ = std::move(cb); }
    void set_ice_callback(ICECallback cb) { ice_callback_ = std::move(cb); }
    void set_state_callback(StateCallback cb) { state_callback_ = std::move(cb); }
    
private:
    /**
     * @brief 创建 webrtc bin 及其内部元素
     */
    bool create_peer_bin();
    
    /**
     * @brief 配置 STUN/TURN 服务器
     */
    void configure_ice_servers();
    
    // GStreamer 回调
    static void on_negotiation_needed(GstElement* webrtc, gpointer user_data);
    static void on_ice_candidate(GstElement* webrtc, guint mline_index, 
                                  gchar* candidate, gpointer user_data);
    static void on_ice_gathering_state_changed(GstElement* webrtc, 
                                                GParamSpec* pspec, 
                                                gpointer user_data);
    static void on_ice_connection_state_changed(GstElement* webrtc,
                                                 GParamSpec* pspec,
                                                 gpointer user_data);
    
    // SDP 回调
    static void on_offer_created(GstPromise* promise, gpointer user_data);
    static void on_answer_created(GstPromise* promise, gpointer user_data);
    
private:
    std::string peer_id_;
    const Config& config_;
    
    GstElement* main_pipeline_ = nullptr; // 主管道引用 (不拥有)
    GstElement* peer_bin_ = nullptr;      // 本 Peer 的 Bin
    GstElement* webrtcbin_ = nullptr;
    GstPad* sink_pad_ = nullptr;          // Bin 的输入 Pad
    
    PeerState state_ = PeerState::IDLE;
    
    SDPCallback sdp_callback_;
    ICECallback ice_callback_;
    StateCallback state_callback_;
};

} // namespace qyh::mediaplane
