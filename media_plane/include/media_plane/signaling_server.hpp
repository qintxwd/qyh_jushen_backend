/**
 * @file signaling_server.hpp
 * @brief WebRTC 信令服务器
 * 
 * 基于 WebSocket 的信令服务器，处理 SDP Offer/Answer 和 ICE Candidate 交换
 */

#pragma once

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <memory>
#include <unordered_map>
#include <mutex>
#include <functional>
#include <string>

namespace qyh::mediaplane {

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

// 前向声明
class Config;
class WebRTCPeer;
class PipelineManager;

/**
 * @brief 信令会话
 */
class SignalingSession : public std::enable_shared_from_this<SignalingSession> {
public:
    SignalingSession(tcp::socket&& socket,
                     class SignalingServer& server);
    
    ~SignalingSession();
    
    void start();
    void close();
    void send(const std::string& message);
    
    const std::string& peer_id() const { return peer_id_; }
    void set_webrtc_peer(std::shared_ptr<WebRTCPeer> peer) { webrtc_peer_ = peer; }
    std::shared_ptr<WebRTCPeer> webrtc_peer() const { return webrtc_peer_; }
    
private:
    void do_accept();
    void on_accept(beast::error_code ec);
    void do_read();
    void on_read(beast::error_code ec, std::size_t bytes);
    void do_write();
    void on_write(beast::error_code ec, std::size_t bytes);
    
    void handle_message(const std::string& message);
    
private:
    websocket::stream<beast::tcp_stream> ws_;
    SignalingServer& server_;
    
    std::string peer_id_;
    std::shared_ptr<WebRTCPeer> webrtc_peer_;
    
    beast::flat_buffer read_buffer_;
    std::queue<std::string> write_queue_;
    std::mutex write_mutex_;
    bool writing_ = false;
};

/**
 * @brief 信令服务器
 */
class SignalingServer : public std::enable_shared_from_this<SignalingServer> {
public:
    using SDPCallback = std::function<void(const std::string& peer_id, 
                                            const std::string& sdp, 
                                            const std::string& type)>;
    using ICECallback = std::function<void(const std::string& peer_id,
                                            const std::string& candidate,
                                            const std::string& sdp_mid,
                                            int sdp_mline_index)>;
    
    SignalingServer(net::io_context& io_context, const Config& config);
    ~SignalingServer();
    
    void start();
    void stop();
    
    void set_pipeline_manager(PipelineManager* manager) { pipeline_manager_ = manager; }
    
    /**
     * @brief 发送 SDP 到指定 peer
     */
    void send_sdp(const std::string& peer_id, 
                  const std::string& sdp, 
                  const std::string& type);
    
    /**
     * @brief 发送 ICE Candidate 到指定 peer
     */
    void send_ice_candidate(const std::string& peer_id,
                            const std::string& candidate,
                            const std::string& sdp_mid,
                            int sdp_mline_index);
    
    void add_session(const std::string& peer_id, std::shared_ptr<SignalingSession> session);
    void remove_session(const std::string& peer_id);
    
    PipelineManager* pipeline_manager() { return pipeline_manager_; }
    
private:
    void do_accept();
    void on_accept(beast::error_code ec, tcp::socket socket);
    
private:
    net::io_context& io_context_;
    tcp::acceptor acceptor_;
    const Config& config_;
    
    std::unordered_map<std::string, std::shared_ptr<SignalingSession>> sessions_;
    std::mutex sessions_mutex_;
    
    PipelineManager* pipeline_manager_ = nullptr;
    
    std::atomic<bool> running_{false};
};

} // namespace qyh::mediaplane
