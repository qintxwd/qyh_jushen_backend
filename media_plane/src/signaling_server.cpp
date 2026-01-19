/**
 * @file signaling_server.cpp
 * @brief WebRTC 信令服务器实现
 */

#include "media_plane/signaling_server.hpp"
#include "media_plane/config.hpp"
#include "media_plane/webrtc_peer.hpp"
#include "media_plane/pipeline_manager.hpp"

#include <nlohmann/json.hpp>

#include <iostream>
#include <random>
#include <sstream>
#include <iomanip>

using json = nlohmann::json;

namespace qyh::mediaplane {

// ==================== SignalingSession ====================

static std::string generate_peer_id() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (int i = 0; i < 8; ++i) {
        ss << std::setw(2) << dis(gen);
    }
    
    return ss.str();
}

SignalingSession::SignalingSession(tcp::socket&& socket, SignalingServer& server)
    : ws_(std::move(socket))
    , server_(server)
    , peer_id_(generate_peer_id())
{
    ws_.binary(false);  // JSON 文本模式
}

SignalingSession::~SignalingSession() {
    server_.remove_session(peer_id_);
}

void SignalingSession::start() {
    server_.add_session(peer_id_, shared_from_this());
    do_accept();
}

void SignalingSession::close() {
    beast::error_code ec;
    ws_.close(websocket::close_code::normal, ec);
}

void SignalingSession::send(const std::string& message) {
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        write_queue_.push(message);
    }
    
    net::post(ws_.get_executor(), [self = shared_from_this()]() {
        std::lock_guard<std::mutex> lock(self->write_mutex_);
        if (!self->writing_ && !self->write_queue_.empty()) {
            self->writing_ = true;
            self->do_write();
        }
    });
}

void SignalingSession::do_accept() {
    ws_.async_accept(
        beast::bind_front_handler(&SignalingSession::on_accept, shared_from_this())
    );
}

void SignalingSession::on_accept(beast::error_code ec) {
    if (ec) {
        std::cerr << "WebSocket accept error: " << ec.message() << std::endl;
        return;
    }
    
    std::cout << "Signaling session established: " << peer_id_ << std::endl;
    
    // 发送欢迎消息
    json welcome;
    welcome["type"] = "welcome";
    welcome["peer_id"] = peer_id_;
    send(welcome.dump());
    
    do_read();
}

void SignalingSession::do_read() {
    ws_.async_read(
        read_buffer_,
        beast::bind_front_handler(&SignalingSession::on_read, shared_from_this())
    );
}

void SignalingSession::on_read(beast::error_code ec, std::size_t bytes) {
    if (ec == websocket::error::closed) {
        std::cout << "Signaling session closed: " << peer_id_ << std::endl;
        return;
    }
    
    if (ec) {
        std::cerr << "Read error: " << ec.message() << std::endl;
        return;
    }
    
    // 处理消息
    std::string message(static_cast<const char*>(read_buffer_.data().data()), bytes);
    handle_message(message);
    
    read_buffer_.consume(bytes);
    do_read();
}

void SignalingSession::do_write() {
    if (write_queue_.empty()) {
        writing_ = false;
        return;
    }
    
    auto& message = write_queue_.front();
    
    ws_.async_write(
        net::buffer(message),
        beast::bind_front_handler(&SignalingSession::on_write, shared_from_this())
    );
}

void SignalingSession::on_write(beast::error_code ec, std::size_t /*bytes*/) {
    if (ec) {
        std::cerr << "Write error: " << ec.message() << std::endl;
        return;
    }
    
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        write_queue_.pop();
    }
    
    do_write();
}

void SignalingSession::handle_message(const std::string& message) {
    try {
        auto msg = json::parse(message);
        std::string type = msg["type"];
        
        if (type == "request_stream") {
            // 请求视频流
            std::string video_source = msg.value("source", "head_camera");
            
            if (server_.pipeline_manager()) {
                webrtc_peer_ = server_.pipeline_manager()->create_peer(peer_id_, video_source);
                if (webrtc_peer_) {
                    // 设置 SDP 回调
                    webrtc_peer_->set_sdp_callback(
                        [this](const std::string& sdp, const std::string& sdp_type) {
                            server_.send_sdp(peer_id_, sdp, sdp_type);
                        });
                    
                    // 设置 ICE 回调
                    webrtc_peer_->set_ice_callback(
                        [this](const std::string& candidate, 
                               const std::string& sdp_mid, 
                               int sdp_mline_index) {
                            server_.send_ice_candidate(peer_id_, candidate, 
                                                       sdp_mid, sdp_mline_index);
                        });
                    
                    // 创建 Offer
                    webrtc_peer_->create_offer();
                }
            }
            
        } else if (type == "answer") {
            // 收到 SDP Answer
            std::string sdp = msg["sdp"];
            if (webrtc_peer_) {
                webrtc_peer_->set_remote_description(sdp, "answer");
            }
            
        } else if (type == "ice_candidate") {
            // 收到 ICE Candidate
            std::string candidate = msg["candidate"];
            std::string sdp_mid = msg["sdp_mid"];
            int sdp_mline_index = msg["sdp_mline_index"];
            
            if (webrtc_peer_) {
                webrtc_peer_->add_ice_candidate(candidate, sdp_mid, sdp_mline_index);
            }
            
        } else if (type == "stop_stream") {
            // 停止流
            if (server_.pipeline_manager()) {
                server_.pipeline_manager()->remove_peer(peer_id_);
            }
            webrtc_peer_.reset();
        }
        
    } catch (const json::exception& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
    }
}

// ==================== SignalingServer ====================

SignalingServer::SignalingServer(net::io_context& io_context, const Config& config)
    : io_context_(io_context)
    , acceptor_(io_context)
    , config_(config)
{
}

SignalingServer::~SignalingServer() {
    stop();
}

void SignalingServer::start() {
    beast::error_code ec;
    
    auto address = net::ip::make_address(config_.server.host, ec);
    if (ec) {
        std::cerr << "Invalid address: " << ec.message() << std::endl;
        return;
    }
    
    tcp::endpoint endpoint{address, config_.server.signaling_port};
    
    acceptor_.open(endpoint.protocol(), ec);
    if (ec) {
        std::cerr << "Failed to open acceptor: " << ec.message() << std::endl;
        return;
    }
    
    acceptor_.set_option(net::socket_base::reuse_address(true), ec);
    acceptor_.bind(endpoint, ec);
    if (ec) {
        std::cerr << "Failed to bind: " << ec.message() << std::endl;
        return;
    }
    
    acceptor_.listen(net::socket_base::max_listen_connections, ec);
    if (ec) {
        std::cerr << "Failed to listen: " << ec.message() << std::endl;
        return;
    }
    
    running_ = true;
    std::cout << "Signaling server listening on " 
              << config_.server.host << ":" << config_.server.signaling_port << std::endl;
    
    do_accept();
}

void SignalingServer::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    beast::error_code ec;
    acceptor_.close(ec);
    
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (auto& [id, session] : sessions_) {
        session->close();
    }
    sessions_.clear();
}

void SignalingServer::do_accept() {
    acceptor_.async_accept(
        net::make_strand(io_context_),
        beast::bind_front_handler(&SignalingServer::on_accept, shared_from_this())
    );
}

void SignalingServer::on_accept(beast::error_code ec, tcp::socket socket) {
    if (ec) {
        if (running_) {
            std::cerr << "Accept error: " << ec.message() << std::endl;
        }
    } else {
        auto session = std::make_shared<SignalingSession>(std::move(socket), *this);
        session->start();
    }
    
    if (running_) {
        do_accept();
    }
}

void SignalingServer::send_sdp(const std::string& peer_id,
                                const std::string& sdp,
                                const std::string& type) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    auto it = sessions_.find(peer_id);
    if (it != sessions_.end()) {
        json msg;
        msg["type"] = type;  // "offer" 或 "answer"
        msg["sdp"] = sdp;
        it->second->send(msg.dump());
    }
}

void SignalingServer::send_ice_candidate(const std::string& peer_id,
                                          const std::string& candidate,
                                          const std::string& sdp_mid,
                                          int sdp_mline_index) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    auto it = sessions_.find(peer_id);
    if (it != sessions_.end()) {
        json msg;
        msg["type"] = "ice_candidate";
        msg["candidate"] = candidate;
        msg["sdp_mid"] = sdp_mid;
        msg["sdp_mline_index"] = sdp_mline_index;
        it->second->send(msg.dump());
    }
}

void SignalingServer::add_session(const std::string& peer_id, 
                                   std::shared_ptr<SignalingSession> session) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_[peer_id] = std::move(session);
}

void SignalingServer::remove_session(const std::string& peer_id) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_.erase(peer_id);
    
    // 清理 WebRTC Peer
    if (pipeline_manager_) {
        pipeline_manager_->remove_peer(peer_id);
    }
}

} // namespace qyh::mediaplane
