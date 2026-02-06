/**
 * @file vr_session.cpp
 * @brief VR 专用会话管理实现
 */

#include "data_plane/vr_session.hpp"
#include "data_plane/session.hpp"
#include "data_plane/logger.hpp"

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <nlohmann/json.hpp>

#include <thread>
#include <sstream>

namespace qyh::dataplane {

namespace {
namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

/**
 * @brief 解析 HTTP URL
 */
struct ParsedUrl {
    std::string host;
    std::string port;
    std::string target;
    bool valid = false;
};

ParsedUrl parse_url(const std::string& url, const std::string& path) {
    ParsedUrl result;
    
    std::string work = url;
    if (work.rfind("http://", 0) == 0) {
        work = work.substr(7);
    }
    
    std::string hostport;
    auto slash_pos = work.find('/');
    if (slash_pos == std::string::npos) {
        hostport = work;
    } else {
        hostport = work.substr(0, slash_pos);
    }
    
    auto colon_pos = hostport.find(':');
    if (colon_pos == std::string::npos) {
        result.host = hostport;
        result.port = "8000";
    } else {
        result.host = hostport.substr(0, colon_pos);
        result.port = hostport.substr(colon_pos + 1);
    }
    
    result.target = path;
    result.valid = !result.host.empty();
    return result;
}

/**
 * @brief 发送 HTTP POST 请求（异步，不阻塞）
 */
void http_post_async(const std::string& url, const std::string& path,
                     const nlohmann::json& body,
                     const std::string& token) {
    // 在独立线程中执行，不阻塞主线程
    std::thread([url, path, body]() {
        auto parsed = parse_url(url, path);
        if (!parsed.valid) {
            LOG_ERROR("[VRSession] Invalid URL: " << url);
            return;
        }
        
        try {
            net::io_context ioc;
            tcp::resolver resolver(ioc);
            beast::tcp_stream stream(ioc);
            
            auto const results = resolver.resolve(parsed.host, parsed.port);
            stream.expires_after(std::chrono::seconds(5));
            stream.connect(results);
            
            http::request<http::string_body> req{
                http::verb::post, parsed.target, 11
            };
            req.set(http::field::host, parsed.host);
            req.set(http::field::content_type, "application/json");
            if (!token.empty()) {
                req.set(http::field::authorization, "Bearer " + token);
            }
            req.body() = body.dump();
            req.prepare_payload();
            
            http::write(stream, req);
            
            beast::flat_buffer buffer;
            http::response<http::string_body> res;
            http::read(stream, buffer, res);
            
            if (res.result() != http::status::ok) {
                LOG_WARN("[VRSession] Control Plane returned " 
                         << res.result_int());
            }
            
            beast::error_code ec;
            stream.socket().shutdown(tcp::socket::shutdown_both, ec);
            
        } catch (const std::exception& e) {
            LOG_ERROR("[VRSession] HTTP POST failed: " << e.what());
        }
    }).detach();
}

} // anonymous namespace

// ==================== VRSessionManager ====================

VRSessionManager& VRSessionManager::instance() {
    static VRSessionManager instance;
    return instance;
}

bool VRSessionManager::try_accept(std::shared_ptr<Session> session,
                                   const std::string& device,
                                   const std::string& version) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查是否已有 VR 连接
    if (connected_) {
        auto existing = vr_session_.lock();
        if (existing) {
            LOG_WARN("[VRSession] Rejecting VR connection: channel occupied by "
                     << client_info_.session_id);
            return false;
        }
        // 旧连接已失效，可以接受新连接
    }
    
    // 接受新连接
    vr_session_ = session;
    client_info_ = VRClientInfo{
        device,
        version,
        session->session_id(),
        std::chrono::system_clock::now()
    };
    connected_ = true;
    
    LOG_INFO("[VRSession] VR connected: " << device << " v" << version
             << " (session: " << session->session_id() << ")");
    
    // 通知 Control Plane
    notify_control_plane(true, client_info_);
    
    // 触发回调
    if (connection_callback_) {
        connection_callback_(true, client_info_);
    }
    
    return true;
}

void VRSessionManager::on_disconnect(const std::string& session_id,
                                      const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 验证是当前 VR 会话
    if (!connected_ || client_info_.session_id != session_id) {
        return;
    }
    
    LOG_INFO("[VRSession] VR disconnected: " << client_info_.device
             << ", reason: " << reason);
    
    VRClientInfo old_info = client_info_;
    
    // 清理状态
    vr_session_.reset();
    client_info_ = VRClientInfo{};
    connected_ = false;
    
    // 通知 Control Plane
    notify_control_plane(false, old_info, reason);
    
    // 触发回调
    if (connection_callback_) {
        connection_callback_(false, old_info);
    }
}

bool VRSessionManager::is_connected() const {
    return connected_;
}

VRClientInfo VRSessionManager::get_client_info() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return client_info_;
}

std::shared_ptr<Session> VRSessionManager::get_session() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vr_session_.lock();
}

bool VRSessionManager::is_vr_session(const std::string& session_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return connected_ && client_info_.session_id == session_id;
}

void VRSessionManager::set_connection_callback(VRConnectionCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    connection_callback_ = std::move(callback);
}

void VRSessionManager::set_control_plane_url(const std::string& url) {
    std::lock_guard<std::mutex> lock(mutex_);
    control_plane_url_ = url;
}

void VRSessionManager::set_internal_token(const std::string& token) {
    std::lock_guard<std::mutex> lock(mutex_);
    internal_token_ = token;
}

void VRSessionManager::notify_control_plane(bool connected,
                                             const VRClientInfo& info,
                                             const std::string& reason) {
    std::string url;
    std::string token;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        url = control_plane_url_;
        token = internal_token_;
    }

    if (url.empty()) {
        return;
    }
    
    if (connected) {
        // POST /internal/vr/connected
        nlohmann::json body;
        body["device"] = info.device;
        body["version"] = info.version;
        body["session_id"] = info.session_id;
        
        http_post_async(url,
                "/api/v1/vr/internal/connected", body, token);
    } else {
        // POST /internal/vr/disconnected
        nlohmann::json body;
        body["session_id"] = info.session_id;
        body["reason"] = reason;
        
        http_post_async(url,
                        "/api/v1/vr/internal/disconnected", body, token);
    }
}

} // namespace qyh::dataplane
