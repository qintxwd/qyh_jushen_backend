/**
 * @file session.cpp
 * @brief WebSocket 会话实现
 */

#include "data_plane/session.hpp"
#include "data_plane/server.hpp"
#include "data_plane/message_handler.hpp"
#include "data_plane/control_sync.hpp"
#include "data_plane/vr_session.hpp"
#include "data_plane/logger.hpp"

#include <random>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace qyh::dataplane {

Session::Session(tcp::socket&& socket, 
                 Server& server,
                 MessageHandler& handler)
    : ws_(std::move(socket))
    , server_(server)
    , handler_(handler)
    , session_id_(generate_session_id())
{
    // 设置 WebSocket 选项
    ws_.binary(true);  // 使用二进制模式
    
    // 设置超时（关键：防止资源泄漏）
    // 这个配置确保网络波动导致的僵死连接会被自动断开
    beast::websocket::stream_base::timeout opt{
        std::chrono::seconds(30),   // 握手超时
        std::chrono::seconds(300),  // 空闲超时（5分钟）
        true                        // 启用 ping/pong 心跳
    };
    ws_.set_option(opt);
    
    // 注：如果客户端 5 分钟无活动，Boost.Beast 会自动发送 ping
    // 如果客户端不响应 pong，连接会被关闭，防止 socket 泄漏
}

Session::~Session() {
    // 清理 VR 会话（如果是 VR 客户端）
    VRSessionManager::instance().on_disconnect(session_id_, "session_closed");
    
    if (control_sync_) {
        control_sync_->disassociate_session(session_id_);
    }
    server_.remove_session(session_id_);
}

void Session::start() {
    // 注册会话
    server_.add_session(session_id_, shared_from_this());
    
    // 执行 WebSocket 握手
    do_accept();
}

void Session::close() {
    if (state_ == SessionState::CLOSING) {
        return;
    }
    
    state_ = SessionState::CLOSING;
    
    beast::error_code ec;
    ws_.close(websocket::close_code::normal, ec);
}

void Session::send(std::shared_ptr<const std::vector<uint8_t>> data) {
    std::cout << "[Session] send() 调用, 数据大小: " << (data ? data->size() : 0) << " 字节" << std::endl;
    if (!data) return;

    bool should_start_write = false;
    
    // 加入发送队列
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        write_queue_.push(data);
        if (!writing_) {
            writing_ = true;
            should_start_write = true;
        }
    }
    
    // 如果没有正在写入，启动写入（在锁外调用）
    if (should_start_write) {
        net::post(ws_.get_executor(), [self = shared_from_this()]() {
            self->do_write();
        });
    }
}

void Session::send(const std::vector<uint8_t>& data) {
    // 兼容接口：创建共享指针（发生一次拷贝）
    auto shared_data = std::make_shared<std::vector<uint8_t>>(data);
    send(shared_data);
}

void Session::subscribe(const std::string& topic) {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    subscriptions_.insert(topic);
}

void Session::unsubscribe(const std::string& topic) {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    subscriptions_.erase(topic);
}

bool Session::is_subscribed(const std::string& topic) const {
    std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    return subscriptions_.count(topic) > 0;
}

void Session::update_heartbeat() {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    last_heartbeat_ = std::chrono::steady_clock::now();
}

bool Session::is_heartbeat_timeout(std::chrono::milliseconds timeout) const {
    std::lock_guard<std::mutex> lock(heartbeat_mutex_);
    auto now = std::chrono::steady_clock::now();
    return (now - last_heartbeat_) > timeout;
}

bool Session::has_control_permission() const {
    if (!user_info_.has_permission("robot:control")) {
        return false;
    }

    if (control_sync_) {
        return control_sync_->session_has_control(session_id_);
    }

    return true;
}

void Session::do_accept() {
    ws_.async_accept(
        beast::bind_front_handler(&Session::on_accept, shared_from_this())
    );
}

void Session::on_accept(beast::error_code ec) {
    if (ec) {
        std::cerr << "WebSocket accept error: " << ec.message() << std::endl;
        return;
    }
    
    std::cout << "WebSocket handshake complete for session: " << session_id_ << std::endl;
    
    // 初始化心跳
    update_heartbeat();
    
    // 开始读取消息
    do_read();
}

void Session::do_read() {
    ws_.async_read(
        read_buffer_,
        beast::bind_front_handler(&Session::on_read, shared_from_this())
    );
}

void Session::on_read(beast::error_code ec, std::size_t bytes_transferred) {
    if (ec == websocket::error::closed) {
        std::cout << "Session " << session_id_ << " closed by client" << std::endl;
        return;
    }
    
    if (ec) {
        std::cerr << "Read error: " << ec.message() << std::endl;
        return;
    }
    
    // 处理消息
    auto data = static_cast<const uint8_t*>(read_buffer_.data().data());
    std::vector<uint8_t> message(data, data + bytes_transferred);
    
    handler_.handle_message(shared_from_this(), message);
    
    // 清空缓冲区
    read_buffer_.consume(bytes_transferred);
    
    // 继续读取
    do_read();
}

void Session::do_write() {
    std::cout << "[Session] do_write() 开始" << std::endl;
    
    std::shared_ptr<const std::vector<uint8_t>> data;
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        if (write_queue_.empty()) {
            writing_ = false;
            std::cout << "[Session] do_write() 队列为空，退出" << std::endl;
            return;
        }
        data = write_queue_.front();
    }
    
    std::cout << "[Session] do_write() 准备发送 " << data->size() << " 字节" << std::endl;
    
    ws_.async_write(
        net::buffer(*data),
        beast::bind_front_handler(&Session::on_write, shared_from_this())
    );
    
    std::cout << "[Session] do_write() async_write已调用" << std::endl;
}

void Session::on_write(beast::error_code ec, std::size_t bytes_transferred) {
    std::cout << "[Session] on_write() 完成, bytes=" << bytes_transferred << ", ec=" << ec.message() << std::endl;
    if (ec) {
        std::cerr << "Write error: " << ec.message() << std::endl;
        std::lock_guard<std::mutex> lock(write_mutex_);
        writing_ = false;
        return;
    }
    
    bool has_more = false;
    
    // 移除已发送的消息
    {
        std::lock_guard<std::mutex> lock(write_mutex_);
        if (!write_queue_.empty()) {
            write_queue_.pop();
        }
        has_more = !write_queue_.empty();
        if (!has_more) {
            writing_ = false;
        }
    }
    
    // 如果还有消息，继续发送
    if (has_more) {
        do_write();
    }
}

std::string Session::generate_session_id() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    
    std::stringstream ss;
    ss << std::hex << std::setfill('0');
    for (int i = 0; i < 16; ++i) {
        ss << std::setw(2) << dis(gen);
    }
    
    return ss.str();
}

} // namespace qyh::dataplane
