/**
 * @file server.cpp
 * @brief WebSocket 服务器实现
 */

#include "data_plane/server.hpp"
#include "data_plane/session.hpp"
#include "data_plane/config.hpp"
#include "data_plane/message_handler.hpp"
#include "data_plane/control_sync.hpp"
#include "data_plane/logger.hpp"

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
        LOG_ERROR("Failed to open acceptor: " << ec.message());
        return;
    }
    
    // 设置 SO_REUSEADDR
    acceptor_.set_option(net::socket_base::reuse_address(true), ec);
    if (ec) {
        LOG_ERROR("Failed to set SO_REUSEADDR: " << ec.message());
        return;
    }
    
    // 绑定地址
    acceptor_.bind(endpoint, ec);
    if (ec) {
        LOG_ERROR("Failed to bind: " << ec.message());
        return;
    }
    
    // 开始监听
    acceptor_.listen(net::socket_base::max_listen_connections, ec);
    if (ec) {
        LOG_ERROR("Failed to listen: " << ec.message());
        return;
    }
    
    running_ = true;
    LOG_INFO("WebSocket server listening on " 
              << config_.server.host << ":" << config_.server.port);
    
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
            LOG_ERROR("Accept error: " << ec.message());
        }
    } else {
        // 检查连接数限制
        if (session_count() >= config_.server.max_connections) {
            LOG_WARN("Max connections reached, rejecting");
            socket.close();
        } else {
            // 创建新会话
            auto session = std::make_shared<Session>(std::move(socket), *this, handler_);
            if (control_sync_) {
                session->set_control_sync(control_sync_);
            }
            session->start();
            LOG_INFO("New connection from " 
                      << socket.remote_endpoint().address().to_string());
        }
    }
    
    // 继续接受新连接
    if (running_) {
        do_accept();
    }
}

void Server::broadcast(std::shared_ptr<const std::vector<uint8_t>> data) {
    if (!data) return;
    
    // Copy-on-Write: 拷贝 session 列表，减小锁粒度
    std::vector<std::shared_ptr<Session>> sessions_copy;
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        sessions_copy.reserve(sessions_.size());
        for (auto& [id, session] : sessions_) {
            if (session->state() == SessionState::AUTHENTICATED ||
                session->state() == SessionState::ACTIVE) {
                sessions_copy.push_back(session);
            }
        }
    }
    
    // 解锁后发送，避免阻塞其他操作
    for (auto& session : sessions_copy) {
        session->send(data);
    }
}

void Server::broadcast(const std::vector<uint8_t>& data) {
    auto shared_data = std::make_shared<std::vector<uint8_t>>(data);
    broadcast(shared_data);
}

void Server::broadcast_to_subscribers(const std::string& topic,
                                       std::shared_ptr<const std::vector<uint8_t>> data) {
    if (!data) return;

    // Copy-on-Write: 拷贝订阅该 topic 的 session 列表
    std::vector<std::shared_ptr<Session>> subscribers;
    {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        for (auto& [id, session] : sessions_) {
            if (session->is_subscribed(topic)) {
                subscribers.push_back(session);
            }
        }
    }
    
    // 解锁后发送
    for (auto& session : subscribers) {
        session->send(data);
    }
}

void Server::broadcast_to_subscribers(const std::string& topic,
                                       const std::vector<uint8_t>& data) {
    auto shared_data = std::make_shared<std::vector<uint8_t>>(data);
    broadcast_to_subscribers(topic, shared_data);
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
