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

#include <algorithm>
#include <iostream>

namespace qyh::dataplane {

Server::Server(net::io_context& io_context, const Config& config, MessageHandler& handler)
    : io_context_(io_context)
    , acceptor_(io_context)
    , config_(config)
    , handler_(handler)
    , connection_manager_([&config]() {
          ConnectionManagerConfig cm_config;
          cm_config.max_connections = static_cast<int>(config.server.max_connections);
          return cm_config;
      }())
{
    // Kick callback: close target session when connection manager decides to evict.
    connection_manager_.set_kick_callback([this](const std::string& session_id,
                                                  const std::string& reason) {
        std::shared_ptr<Session> session;
        {
            std::lock_guard<std::mutex> lock(sessions_mutex_);
            auto it = sessions_.find(session_id);
            if (it != sessions_.end()) {
                session = it->second;
            }
        }

        if (!session) {
            LOG_WARN("Kick requested for unknown session " << session_id
                     << ", reason=" << reason);
            return;
        }

        LOG_WARN("Kicking session " << session_id << ", reason=" << reason);
        session->close();
    });
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
    
    // 关闭所有会话（避免持锁调用 close 导致死锁）
    std::vector<std::shared_ptr<Session>> sessions_copy;
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
        // 解析远端地址
        beast::error_code ep_ec;
        auto remote = socket.remote_endpoint(ep_ec);
        std::string remote_str = ep_ec ? "unknown" : remote.address().to_string();

        // 连接治理：限流/黑名单/优先级驱逐
        auto accept_result = connection_manager_.try_accept(remote_str, "unknown");
        if (!accept_result.accepted) {
            LOG_WARN("Rejecting connection from " << remote_str
                     << ": " << accept_result.message);
            socket.close();
        } else {
            // 创建新会话
            auto session = std::make_shared<Session>(
                std::move(socket),
                *this,
                handler_,
                remote_str,
                &connection_manager_,
                std::chrono::seconds(std::max(0, config_.auth.auth_timeout_sec))
            );
            if (control_sync_) {
                session->set_control_sync(control_sync_);
            }
            session->start();
            LOG_INFO("New connection from " << remote_str);
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
    
    static int broadcast_count = 0;
    if (++broadcast_count % 100 == 1) {
        std::cout << "[Server] broadcast topic=" << topic 
                  << ", subscribers=" << subscribers.size() 
                  << ", data_size=" << data->size() << std::endl;
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
