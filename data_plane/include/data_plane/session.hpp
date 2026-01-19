/**
 * @file session.hpp
 * @brief WebSocket 会话管理
 */

#pragma once

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>

#include <memory>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <mutex>
#include <chrono>
#include <functional>

namespace qyh::dataplane {

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

// 前向声明
class Server;
class MessageHandler;

/**
 * @brief 会话状态
 */
enum class SessionState {
    CONNECTING,     // 连接中（等待鉴权）
    AUTHENTICATED,  // 已认证
    ACTIVE,         // 活跃（已订阅）
    CLOSING         // 关闭中
};

/**
 * @brief 用户信息（从 JWT 解析）
 */
struct UserInfo {
    int64_t user_id = 0;
    std::string username;
    std::string role;
    std::vector<std::string> permissions;
    
    bool has_permission(const std::string& perm) const {
        return std::find(permissions.begin(), permissions.end(), perm) != permissions.end();
    }
};

/**
 * @brief WebSocket 会话
 */
class Session : public std::enable_shared_from_this<Session> {
public:
    /**
     * @brief 构造函数
     * @param socket TCP 套接字
     * @param server 服务器引用
     * @param handler 消息处理器
     */
    Session(tcp::socket&& socket, 
            Server& server,
            MessageHandler& handler);
    
    ~Session();
    
    /**
     * @brief 启动会话
     */
    void start();
    
    /**
     * @brief 关闭会话
     */
    void close();
    
    /**
     * @brief 发送消息
     * @param data 消息数据
     */
    void send(const std::vector<uint8_t>& data);
    
    /**
     * @brief 获取会话 ID
     */
    const std::string& session_id() const { return session_id_; }
    
    /**
     * @brief 获取会话状态
     */
    SessionState state() const { return state_; }
    
    /**
     * @brief 设置会话状态
     */
    void set_state(SessionState state) { state_ = state; }
    
    /**
     * @brief 获取用户信息
     */
    const UserInfo& user_info() const { return user_info_; }
    
    /**
     * @brief 设置用户信息（认证成功后）
     */
    void set_user_info(const UserInfo& info) { user_info_ = info; }
    
    /**
     * @brief 添加订阅
     */
    void subscribe(const std::string& topic);
    
    /**
     * @brief 取消订阅
     */
    void unsubscribe(const std::string& topic);
    
    /**
     * @brief 检查是否订阅了某话题
     */
    bool is_subscribed(const std::string& topic) const;
    
    /**
     * @brief 获取所有订阅
     */
    const std::set<std::string>& subscriptions() const { return subscriptions_; }
    
    /**
     * @brief 更新心跳
     */
    void update_heartbeat();
    
    /**
     * @brief 检查心跳是否超时
     * @param timeout 超时时间
     */
    bool is_heartbeat_timeout(std::chrono::milliseconds timeout) const;
    
    /**
     * @brief 是否有控制权限
     */
    bool has_control_permission() const;
    
private:
    /**
     * @brief 执行 WebSocket 握手
     */
    void do_accept();
    
    /**
     * @brief 处理握手完成
     */
    void on_accept(beast::error_code ec);
    
    /**
     * @brief 读取消息
     */
    void do_read();
    
    /**
     * @brief 处理读取完成
     */
    void on_read(beast::error_code ec, std::size_t bytes_transferred);
    
    /**
     * @brief 写入消息
     */
    void do_write();
    
    /**
     * @brief 处理写入完成
     */
    void on_write(beast::error_code ec, std::size_t bytes_transferred);
    
    /**
     * @brief 生成会话 ID
     */
    static std::string generate_session_id();
    
private:
    websocket::stream<beast::tcp_stream> ws_;
    Server& server_;
    MessageHandler& handler_;
    
    std::string session_id_;
    SessionState state_ = SessionState::CONNECTING;
    UserInfo user_info_;
    
    beast::flat_buffer read_buffer_;
    
    std::queue<std::vector<uint8_t>> write_queue_;
    std::mutex write_mutex_;
    bool writing_ = false;
    
    std::set<std::string> subscriptions_;
    mutable std::mutex subscriptions_mutex_;
    
    std::chrono::steady_clock::time_point last_heartbeat_;
    mutable std::mutex heartbeat_mutex_;
};

} // namespace qyh::dataplane
