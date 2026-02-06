/**
 * @file config.hpp
 * @brief 配置管理
 */

#pragma once

#include <string>
#include <cstdint>

namespace qyh::dataplane {

/**
 * @brief 服务器配置
 */
struct ServerConfig {
    std::string host = "0.0.0.0";
    uint16_t port = 8765;
    size_t max_connections = 50;
    size_t recv_buffer_size = 65536;
    size_t send_buffer_size = 65536;
};

/**
 * @brief 认证配置
 */
struct AuthConfig {
    std::string jwt_secret;
    std::string jwt_algorithm = "HS256";
    std::string auth_audience = "data_plane";
    std::string reject_scope = "media";
    int auth_timeout_sec = 10;
    bool enabled = true;
};

/**
 * @brief ROS2 配置
 */
struct ROS2Config {
    int domain_id = 0;
    std::string node_name = "ws_data_plane";
};

/**
 * @brief Watchdog 配置
 */
struct WatchdogConfig {
    int timeout_ms = 200;
    int check_interval_ms = 50;
};

/**
 * @brief 状态推送配置
 */
struct StatePublishConfig {
    int robot_state_hz = 30;
    int joint_state_hz = 100;
    bool enable_aggregation = true;
};

/**
 * @brief 控制权同步配置
 */
struct ControlSyncConfig {
    std::string control_plane_url = "http://127.0.0.1:8000";
    std::string internal_token;
    int sync_interval_ms = 1000;
    int timeout_ms = 5000;
    bool enabled = true;
};

/**
 * @brief 日志配置
 */
struct LoggingConfig {
    std::string level = "INFO";
    std::string file;
    bool journald = true;
};

/**
 * @brief 总配置
 */
class Config {
public:
    /**
     * @brief 从文件加载配置
     * @param path 配置文件路径
     * @return 是否成功
     */
    bool load_from_file(const std::string& path);
    
    /**
     * @brief 从环境变量覆盖配置
     */
    void load_from_env();
    
    // 各模块配置
    ServerConfig server;
    AuthConfig auth;
    ROS2Config ros2;
    WatchdogConfig watchdog;
    StatePublishConfig state_publish;
    ControlSyncConfig control_sync;
    LoggingConfig logging;
    
private:
    /**
     * @brief 展开环境变量
     * @param value 可能包含 ${VAR} 的字符串
     * @return 展开后的字符串
     */
    static std::string expand_env(const std::string& value);
};

} // namespace qyh::dataplane
