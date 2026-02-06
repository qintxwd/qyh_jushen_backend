/**
 * @file signaling_server.cpp
 * @brief WebRTC 信令服务器实现
 */

#include "media_plane/signaling_server.hpp"
#include "media_plane/config.hpp"
#include "media_plane/webrtc_peer.hpp"
#include "media_plane/pipeline_manager.hpp"
#include "media_plane/auth.hpp"

#include <nlohmann/json.hpp>

#include <chrono>
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
    , auth_timer_(ws_.get_executor())
{
    ws_.binary(false);  // JSON 文本模式
    ws_.read_message_max(server_.max_message_bytes());
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
    auth_timer_.cancel(ec);
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

void SignalingSession::send_error(const std::string& error, const std::string& details) {
    json msg;
    msg["type"] = "error";
    msg["error"] = error;
    if (!details.empty()) {
        msg["details"] = details;
    }
    send(msg.dump());
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
    welcome["require_auth"] = server_.require_auth();
    welcome["available_sources"] = server_.get_available_sources();
    send(welcome.dump());

    if (server_.require_auth() && server_.auth_timeout_sec() > 0) {
        auth_timer_.expires_after(std::chrono::seconds(server_.auth_timeout_sec()));
        auth_timer_.async_wait([self = shared_from_this()](const beast::error_code& timer_ec) {
            if (timer_ec) {
                return;
            }
            if (!self->authenticated_) {
                self->send_error("auth_timeout", "Authentication timed out");
                self->close();
            }
        });
    }
    
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
        std::string type = msg.value("type", "");
        
        if (type.empty()) {
            send_error("invalid_message", "Missing 'type' field");
            return;
        }
        
        // 认证消息
        if (type == "auth") {
            std::string token = msg.value("token", "");
            if (token.empty()) {
                send_error("auth_failed", "Missing token");
                return;
            }
            
            std::string user_id, username;
            if (server_.verify_token(token, user_id, username)) {
                mark_authenticated();
                user_id_ = user_id;
                username_ = username;
                
                json response;
                response["type"] = "auth_success";
                response["user_id"] = user_id_;
                response["username"] = username_;
                send(response.dump());
                
                std::cout << "Session " << peer_id_ << " authenticated as " << username_ << std::endl;
            } else {
                send_error("auth_failed", "Invalid or expired token");
            }
            return;
        }
        
        // 检查认证状态（对于需要认证的操作）
        if (server_.require_auth() && !authenticated_) {
            if (type == "request_stream" || type == "stop_stream" || 
                type == "switch_source" || type == "get_sources") {
                send_error("unauthorized", "Authentication required");
                return;
            }
        }
        
        if (type == "request_stream") {
            // 请求视频流
            std::string video_source = msg.value("source", "head_camera");
            
            if (!server_.pipeline_manager()) {
                send_error("internal_error", "Pipeline manager not available");
                return;
            }
            
            webrtc_peer_ = server_.pipeline_manager()->create_peer(peer_id_, video_source);
            if (!webrtc_peer_) {
                send_error("stream_error", "Failed to create WebRTC peer");
                return;
            }
            
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
            
            std::cout << "Stream requested by peer " << peer_id_ 
                      << " for source: " << video_source << std::endl;
            
        } else if (type == "answer") {
            // 收到 SDP Answer
            if (!webrtc_peer_) {
                send_error("stream_error", "No active stream");
                return;
            }
            
            std::string sdp = msg.value("sdp", "");
            if (sdp.empty()) {
                send_error("invalid_message", "Missing SDP");
                return;
            }
            
            webrtc_peer_->set_remote_description(sdp, "answer");
            
        } else if (type == "ice_candidate") {
            // 收到 ICE Candidate
            if (!webrtc_peer_) {
                // ICE 可能在流停止后到达，忽略即可
                return;
            }
            
            std::string candidate = msg.value("candidate", "");
            std::string sdp_mid = msg.value("sdp_mid", "");
            int sdp_mline_index = msg.value("sdp_mline_index", 0);
            
            webrtc_peer_->add_ice_candidate(candidate, sdp_mid, sdp_mline_index);
            
        } else if (type == "stop_stream") {
            // 停止流
            if (server_.pipeline_manager()) {
                server_.pipeline_manager()->remove_peer(peer_id_);
            }
            webrtc_peer_.reset();
            
            json response;
            response["type"] = "stream_stopped";
            send(response.dump());
            
            std::cout << "Stream stopped for peer " << peer_id_ << std::endl;
            
        } else if (type == "switch_source") {
            // 切换视频源
            std::string new_source = msg.value("source", "");
            if (new_source.empty()) {
                send_error("invalid_message", "Missing source");
                return;
            }
            
            if (!server_.pipeline_manager()) {
                send_error("internal_error", "Pipeline manager not available");
                return;
            }
            
            // 检查源是否可用
            if (!server_.pipeline_manager()->is_source_available(new_source)) {
                send_error("invalid_source", "Source not available: " + new_source);
                return;
            }
            
            // 目前实现：停止当前流并通知客户端重新请求
            // 未来可以实现无缝切换
            if (webrtc_peer_) {
                server_.pipeline_manager()->remove_peer(peer_id_);
                webrtc_peer_.reset();
            }
            
            json response;
            response["type"] = "source_switched";
            response["source"] = new_source;
            response["reconnect_required"] = true;
            response["message"] = "Please request stream again with new source";
            send(response.dump());
            
            std::cout << "Source switch requested for peer " << peer_id_ 
                      << " to: " << new_source << std::endl;
            
        } else if (type == "get_sources") {
            // 获取可用视频源列表
            json response;
            response["type"] = "sources";
            response["sources"] = server_.get_available_sources();
            send(response.dump());
            
        } else if (type == "ping") {
            // Ping/Pong 保活
            json response;
            response["type"] = "pong";
            response["timestamp"] = msg.value("timestamp", 0);
            send(response.dump());
            
        } else if (type == "get_stats") {
            // 获取统计信息（需要认证）
            if (server_.require_auth() && !authenticated_) {
                send_error("unauthorized", "Authentication required for stats");
                return;
            }
            
            json response;
            response["type"] = "stats";
            response["connections"] = server_.connection_count();
            response["available_sources"] = server_.get_available_sources();
            
            if (server_.pipeline_manager()) {
                auto stats = server_.pipeline_manager()->get_stats();
                response["active_peers"] = stats.active_peers;
                response["total_peers_created"] = stats.total_peers_created;
                response["error_count"] = stats.error_count;
            }
            
            send(response.dump());
            
        } else {
            send_error("unknown_message", "Unknown message type: " + type);
        }
        
    } catch (const json::exception& e) {
        std::cerr << "JSON parse error: " << e.what() << std::endl;
        send_error("invalid_json", e.what());
    } catch (const std::exception& e) {
        std::cerr << "Error handling message: " << e.what() << std::endl;
        send_error("internal_error", e.what());
    }
}

void SignalingSession::mark_authenticated() {
    authenticated_ = true;
    beast::error_code ec;
    auth_timer_.cancel(ec);
}

// ==================== SignalingServer ====================

SignalingServer::SignalingServer(net::io_context& io_context, const Config& config)
    : io_context_(io_context)
    , acceptor_(io_context)
    , config_(config)
{
    // 创建 JWT 验证器
    std::string jwt_secret = config_.server.jwt_secret;
    require_auth_ = config_.server.require_auth;
    if (!jwt_secret.empty()) {
        jwt_verifier_ = std::make_unique<JwtVerifier>(
            jwt_secret,
            config_.server.auth_audience,
            config_.server.auth_issuer,
            config_.server.auth_scope
        );
    }
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
              << config_.server.host << ":" << config_.server.signaling_port;
    if (require_auth_) {
        std::cout << " (authentication required)";
    }
    std::cout << std::endl;
    
    do_accept();
}

void SignalingServer::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    beast::error_code ec;
    acceptor_.close(ec);

    std::vector<std::shared_ptr<SignalingSession>> sessions_copy;
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        sessions_copy.reserve(sessions_.size());
        for (auto& [id, session] : sessions_) {
            sessions_copy.push_back(session);
        }
    }
    for (auto& session : sessions_copy) {
        session->close();
    }
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        sessions_.clear();
    }
}

bool SignalingServer::verify_token(const std::string& token, 
                                    std::string& user_id, 
                                    std::string& username) {
    if (!jwt_verifier_) {
        return false;
    }
    
        auto user_info_opt = jwt_verifier_->verify(token);
        if (user_info_opt.has_value()) {
                user_id = user_info_opt->user_id;
                username = user_info_opt->username;
        return true;
    }
    
    return false;
}

size_t SignalingServer::connection_count() const {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    return sessions_.size();
}

std::vector<std::string> SignalingServer::get_available_sources() const {
    // 返回配置中的视频源列表
    std::vector<std::string> sources;
    for (const auto& source : config_.video.sources) {
        sources.push_back(source.name);
    }
    
    // 如果没有配置源，返回默认值
    if (sources.empty()) {
        sources.push_back("head_camera");
        sources.push_back("test_pattern");
    }
    
    return sources;
}

void SignalingServer::do_accept() {
    acceptor_.async_accept(
        beast::bind_front_handler(&SignalingServer::on_accept, shared_from_this())
    );
}

void SignalingServer::on_accept(beast::error_code ec, tcp::socket socket) {
    if (ec) {
        if (running_) {
            std::cerr << "Accept error: " << ec.message() << std::endl;
        }
    } else {
        if (connection_count() >= max_connections()) {
            socket.close();
        } else {
        auto session = std::make_shared<SignalingSession>(std::move(socket), *this);
        session->start();
        }
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
