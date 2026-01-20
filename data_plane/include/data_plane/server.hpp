/**
 * @file server.hpp
 * @brief WebSocket 服务器
 * 
 * 基于 Boost.Beast 实现的高性能 WebSocket 服务器
 */

#pragma once

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>

#include <memory>
#include <unordered_map>
#include <mutex>
#include <functional>
#include <string>
#include <vector>

namespace qyh::dataplane {

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

// 前向声明
class Session;
class Config;
class MessageHandler;
class ControlSyncService;

/**
 * @brief WebSocket 服务器
 */
class Server : public std::enable_shared_from_this<Server> {
public:
    /**
     * @brief 构造函数
     * @param io_context IO 上下文
     * @param config 配置
     */
    Server(net::io_context& io_context, const Config& config, MessageHandler& handler);
    
    ~Server();
    
    /**
     * @brief 启动服务器
     */
    void start();
    
    /**
     * @brief 停止服务器
     */
    void stop();
    
    /**
     * @brief 广播消息到所有会话 (零拷贝)
     */
    void broadcast(std::shared_ptr<const std::vector<uint8_t>> data);

    /**
     * @brief 广播到特定话题的订阅者 (零拷贝)
     */
    void broadcast_to_subscribers(const std::string& topic, 
                                  std::shared_ptr<const std::vector<uint8_t>> data);


    /**
     * @brief 广播消息到所有会话 (兼容)
     * @param data 消息数据
     */
    void broadcast(const std::vector<uint8_t>& data);
    
    /**
     * @brief 广播到特定话题的订阅者 (兼容)
     * @param topic 话题名称
     * @param data 消息数据
     */
    void broadcast_to_subscribers(const std::string& topic, 
                                  const std::vector<uint8_t>& data);
    
    /**
     * @brief 添加会话
     * @param session_id 会话 ID
     * @param session 会话指针
     */
    void add_session(const std::string& session_id, 
                     std::shared_ptr<Session> session);
    
    /**
     * @brief 移除会话
     * @param session_id 会话 ID
     */
    void remove_session(const std::string& session_id);
    
    /**
     * @brief 获取会话数量
     */
    size_t session_count() const;

    /**
     * @brief 设置控制权同步服务
     */
    void set_control_sync(ControlSyncService* service) { control_sync_ = service; }
    
private:
    /**
     * @brief 接受新连接
     */
    void do_accept();
    
    /**
     * @brief 处理新连接
     */
    void on_accept(beast::error_code ec, tcp::socket socket);
    
private:
    net::io_context& io_context_;
    tcp::acceptor acceptor_;
    const Config& config_;
    MessageHandler& handler_;

    ControlSyncService* control_sync_ = nullptr;
    
    std::unordered_map<std::string, std::shared_ptr<Session>> sessions_;
    mutable std::mutex sessions_mutex_;
    
    bool running_ = false;
};

} // namespace qyh::dataplane
