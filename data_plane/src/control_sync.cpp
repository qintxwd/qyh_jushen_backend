/**
 * @file control_sync.cpp
 * @brief 控制权同步服务实现
 */

#include "data_plane/control_sync.hpp"
#include "data_plane/logger.hpp"

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <nlohmann/json.hpp>

#include <iostream>

// 简单的 HTTP 客户端（实际项目中可使用 cpr 或 Boost.Beast）
// 这里提供一个模拟实现，生产环境需要替换为真正的 HTTP 客户端

namespace qyh::dataplane {

namespace {
namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

struct ParsedUrl {
    std::string host;
    std::string port;
    std::string target;
    bool valid = false;
};

ParsedUrl parse_http_url(const std::string& url, const std::string& default_path) {
    ParsedUrl result;

    std::string work = url;
    if (work.rfind("http://", 0) == 0) {
        work = work.substr(7);
    }

    // 拆分 host[:port] 和 path
    std::string hostport;
    std::string path;
    auto slash_pos = work.find('/');
    if (slash_pos == std::string::npos) {
        hostport = work;
        path = default_path;
    } else {
        hostport = work.substr(0, slash_pos);
        path = work.substr(slash_pos);
    }

    std::string host;
    std::string port = "80";
    auto colon_pos = hostport.find(':');
    if (colon_pos == std::string::npos) {
        host = hostport;
    } else {
        host = hostport.substr(0, colon_pos);
        port = hostport.substr(colon_pos + 1);
    }

    if (host.empty()) {
        return result;
    }

    if (path.empty()) {
        path = default_path;
    }

    result.host = host;
    result.port = port;
    result.target = path;
    result.valid = true;
    return result;
}

} // namespace

// ==================== ControlSyncService ====================

ControlSyncService::ControlSyncService(const Config& config)
    : config_(config)
{
}

ControlSyncService::~ControlSyncService() {
    stop();
}

void ControlSyncService::start() {
    if (running_ || !config_.enabled) {
        return;
    }
    
    running_ = true;
    sync_thread_ = std::thread(&ControlSyncService::sync_loop, this);
    
    LOG_INFO("[ControlSync] Started with interval " 
              << config_.sync_interval_ms << "ms");
}

void ControlSyncService::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    if (sync_thread_.joinable()) {
        sync_thread_.join();
    }
    
    LOG_INFO("[ControlSync] Stopped");
}

void ControlSyncService::sync_now() {
    fetch_control_status();
}

ControlInfo ControlSyncService::get_control_info() const {
    std::lock_guard<std::mutex> lock(info_mutex_);
    return control_info_;
}

bool ControlSyncService::has_control(int64_t user_id) const {
    std::lock_guard<std::mutex> lock(info_mutex_);
    
    if (!control_info_.is_held) {
        return false;
    }
    
    if (control_info_.is_expired()) {
        return false;
    }
    
    return control_info_.holder_user_id == user_id;
}

bool ControlSyncService::session_has_control(const std::string& session_id) const {
    // 先查找会话对应的用户
    int64_t user_id = 0;
    {
        std::lock_guard<std::mutex> lock(session_mutex_);
        auto it = session_user_map_.find(session_id);
        if (it == session_user_map_.end()) {
            return false;
        }
        user_id = it->second;
    }
    
    return has_control(user_id);
}

void ControlSyncService::on_control_change(ControlChangeCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callbacks_.push_back(std::move(callback));
}

void ControlSyncService::update_control(const ControlInfo& info) {
    ControlInfo old_info;
    {
        std::lock_guard<std::mutex> lock(info_mutex_);
        old_info = control_info_;
        control_info_ = info;
    }
    
    // 检查是否有变化
    if (old_info.is_held != info.is_held ||
        old_info.holder_user_id != info.holder_user_id) {
        notify_change(old_info, info);
    }
}

void ControlSyncService::associate_session(const std::string& session_id, 
                                            int64_t user_id) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    session_user_map_[session_id] = user_id;
    
    LOG_INFO("[ControlSync] Associated session " << session_id 
              << " to user " << user_id);
}

void ControlSyncService::disassociate_session(const std::string& session_id) {
    std::lock_guard<std::mutex> lock(session_mutex_);
    session_user_map_.erase(session_id);
}

void ControlSyncService::sync_loop() {
    while (running_) {
        fetch_control_status();
        
        // 等待下一次同步
        for (int i = 0; i < config_.sync_interval_ms / 100 && running_; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

bool ControlSyncService::fetch_control_status() {
    const std::string default_path = "/api/v1/control/status";
    auto parsed = parse_http_url(config_.control_plane_url, default_path);
    if (!parsed.valid) {
        LOG_ERROR("[ControlSync] Invalid control_plane_url: " << config_.control_plane_url);
        return false;
    }

    try {
        net::io_context ioc;
        tcp::resolver resolver(ioc);
        beast::tcp_stream stream(ioc);

        // 解析并连接
        auto const results = resolver.resolve(parsed.host, parsed.port);
        stream.expires_after(std::chrono::milliseconds(config_.timeout_ms));
        stream.connect(results);

        // 发送请求
        http::request<http::string_body> req{http::verb::get, parsed.target, 11};
        req.set(http::field::host, parsed.host);
        req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
        if (!config_.internal_token.empty()) {
            req.set(http::field::authorization, "Bearer " + config_.internal_token);
        }

        http::write(stream, req);

        // 读取响应
        beast::flat_buffer buffer;
        http::response<http::string_body> res;
        http::read(stream, buffer, res);

        // 关闭连接
        beast::error_code ec;
        stream.socket().shutdown(tcp::socket::shutdown_both, ec);

        if (res.result() != http::status::ok) {
            LOG_ERROR("[ControlSync] HTTP error: " << res.result_int());
            return false;
        }

        auto json = nlohmann::json::parse(res.body(), nullptr, false);
        if (json.is_discarded()) {
            LOG_ERROR("[ControlSync] Invalid JSON response");
            return false;
        }

        if (!json.value("success", false)) {
            LOG_ERROR("[ControlSync] Response error: " << json.value("message", "unknown"));
            return false;
        }

        auto data = json.value("data", nlohmann::json::object());
        bool locked = data.value("locked", false);

        ControlInfo info;
        info.is_held = locked;

        if (locked && data.contains("holder") && data["holder"].is_object()) {
            auto holder = data["holder"];
            info.holder_user_id = holder.value("user_id", 0);
            info.holder_username = holder.value("username", "");
            info.holder_session_id = holder.value("session_id", "");

            int remaining = holder.value("remaining_seconds", 0);
            info.acquired_at = std::chrono::steady_clock::now();
            info.expires_at = info.acquired_at + std::chrono::seconds(remaining);
        }

        update_control(info);
        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("[ControlSync] Exception: " << e.what());
        return false;
    }
}

void ControlSyncService::notify_change(const ControlInfo& old_info, 
                                        const ControlInfo& new_info) {
    std::vector<ControlChangeCallback> callbacks_copy;
    {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callbacks_copy = callbacks_;
    }
    
    for (const auto& cb : callbacks_copy) {
        try {
            cb(old_info, new_info);
        } catch (const std::exception& e) {
            LOG_ERROR("[ControlSync] Callback error: " << e.what());
        }
    }
    
    LOG_INFO("[ControlSync] Control changed: "
              << (new_info.is_held ? new_info.holder_username : "none"));
}

} // namespace qyh::dataplane
