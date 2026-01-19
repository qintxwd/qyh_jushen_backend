/**
 * @file server.cpp
 * @brief WebSocket 服务器实现
 */

#include "data_plane/server.hpp"
#include "data_plane/session.hpp"
#include "data_plane/config.hpp"
#include "data_plane/message_handler.hpp"
#include "data_plane/control_sync.hpp"

#include <iostream>

namespace qyh::dataplane {

Server::Server(net::io_context& io_context, const Config& config, MessageHandler& handler)
    : io_context_(io_context)
    , acceptor_(io_context)
    , config_(config)
    , handler_(handler)
{
}

Server::~Server() {
    stop();
}

void Server::start() {
    beast::error_code ec;
    
    auto const address = net::ip::make_address(config_.server.host, ec);
    if (ec) {
        std::cerr << "Invalid address: " << ec.message() << std::endl;
        return;
    }
    
    tcp::endpoint endpoint{address, config_.server.port};
    
    // 打开 acceptor
    acceptor_.open(endpoint.protocol(), ec);
    if (ec) {
        std::cerr << "Failed to open acceptor: " << ec.message() << std::endl;
        return;
    }
    
    // 设置 SO_REUSEADDR
    acceptor_.set_option(net::socket_base::reuse_address(true), ec);
    if (ec) {
        std::cerr << "Failed to set SO_REUSEADDR: " << ec.message() << std::endl;
        return;
    }
    
    // 绑定地址
    acceptor_.bind(endpoint, ec);
    if (ec) {
        std::cerr << "Failed to bind: " << ec.message() << std::endl;
        return;
    }
    
    // 开始监听
    acceptor_.listen(net::socket_base::max_listen_connections, ec);
    if (ec) {
        std::cerr << "Failed to listen: " << ec.message() << std::endl;
        return;
    }
    
    running_ = true;
    std::cout << "WebSocket server listening on " 
              << config_.server.host << ":" << config_.server.port << std::endl;
    
    do_accept();
}

void Server::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    beast::error_code ec;
    acceptor_.close(ec);
    
    // 关闭所有会话
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (auto& [id, session] : sessions_) {
        session->close();
    }
    sessions_.clear();
}

void Server::do_accept() {
    acceptor_.async_accept(
        net::make_strand(io_context_),
        beast::bind_front_handler(&Server::on_accept, shared_from_this())
    );
}

void Server::on_accept(beast::error_code ec, tcp::socket socket) {
    if (ec) {
        if (running_) {
            std::cerr << "Accept error: " << ec.message() << std::endl;
        }
    } else {
        // 检查连接数限制
        if (session_count() >= config_.server.max_connections) {
            std::cerr << "Max connections reached, rejecting" << std::endl;
            socket.close();
        } else {
            // 创建新会话
            auto session = std::make_shared<Session>(std::move(socket), *this, handler_);
            if (control_sync_) {
                session->set_control_sync(control_sync_);
            }
            session->start();
            std::cout << "New connection from " 
                      << socket.remote_endpoint().address().to_string() << std::endl;
        }
    }
    
    // 继续接受新连接
    if (running_) {
        do_accept();
    }
}

void Server::broadcast(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (auto& [id, session] : sessions_) {
        if (session->state() == SessionState::AUTHENTICATED ||
            session->state() == SessionState::ACTIVE) {
            session->send(data);
        }
    }
}

void Server::broadcast_to_subscribers(const std::string& topic,
                                       const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (auto& [id, session] : sessions_) {
        if (session->is_subscribed(topic)) {
            session->send(data);
        }
    }
}

void Server::add_session(const std::string& session_id, 
                          std::shared_ptr<Session> session) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_[session_id] = std::move(session);
}

void Server::remove_session(const std::string& session_id) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_.erase(session_id);
}

size_t Server::session_count() const {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    return sessions_.size();
}

} // namespace qyh::dataplane
