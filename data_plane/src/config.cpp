/**
 * @file config.cpp
 * @brief 配置管理实现
 */

#include "data_plane/config.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cstdlib>
#include <regex>
#include <iostream>

namespace qyh::dataplane {

bool Config::load_from_file(const std::string& path) {
    try {
        std::ifstream file(path);
        if (!file.is_open()) {
            return false;
        }
        
        YAML::Node config = YAML::LoadFile(path);
        
        // Server 配置
        if (config["server"]) {
            auto s = config["server"];
            if (s["host"]) server.host = s["host"].as<std::string>();
            if (s["port"]) server.port = s["port"].as<uint16_t>();
            if (s["max_connections"]) server.max_connections = s["max_connections"].as<size_t>();
            if (s["recv_buffer_size"]) server.recv_buffer_size = s["recv_buffer_size"].as<size_t>();
            if (s["send_buffer_size"]) server.send_buffer_size = s["send_buffer_size"].as<size_t>();
        }
        
        // Auth 配置
        if (config["auth"]) {
            auto a = config["auth"];
            if (a["jwt_secret"]) auth.jwt_secret = expand_env(a["jwt_secret"].as<std::string>());
            if (a["jwt_algorithm"]) auth.jwt_algorithm = a["jwt_algorithm"].as<std::string>();
            if (a["auth_timeout_sec"]) auth.auth_timeout_sec = a["auth_timeout_sec"].as<int>();
            if (a["enabled"]) auth.enabled = a["enabled"].as<bool>();
        }
        
        // ROS2 配置
        if (config["ros2"]) {
            auto r = config["ros2"];
            if (r["domain_id"]) ros2.domain_id = r["domain_id"].as<int>();
            if (r["node_name"]) ros2.node_name = r["node_name"].as<std::string>();
        }
        
        // Watchdog 配置
        if (config["watchdog"]) {
            auto w = config["watchdog"];
            if (w["timeout_ms"]) watchdog.timeout_ms = w["timeout_ms"].as<int>();
            if (w["check_interval_ms"]) watchdog.check_interval_ms = w["check_interval_ms"].as<int>();
        }
        
        // State Publish 配置
        if (config["state_publish"]) {
            auto sp = config["state_publish"];
            if (sp["robot_state_hz"]) state_publish.robot_state_hz = sp["robot_state_hz"].as<int>();
            if (sp["joint_state_hz"]) state_publish.joint_state_hz = sp["joint_state_hz"].as<int>();
            if (sp["enable_aggregation"]) state_publish.enable_aggregation = sp["enable_aggregation"].as<bool>();
        }

        // Control Sync 配置
        if (config["control_sync"]) {
            auto cs = config["control_sync"];
            if (cs["control_plane_url"]) control_sync.control_plane_url = cs["control_plane_url"].as<std::string>();
            if (cs["internal_token"]) control_sync.internal_token = expand_env(cs["internal_token"].as<std::string>());
            if (cs["sync_interval_ms"]) control_sync.sync_interval_ms = cs["sync_interval_ms"].as<int>();
            if (cs["timeout_ms"]) control_sync.timeout_ms = cs["timeout_ms"].as<int>();
            if (cs["enabled"]) control_sync.enabled = cs["enabled"].as<bool>();
        }
        
        // Logging 配置
        if (config["logging"]) {
            auto l = config["logging"];
            if (l["level"]) logging.level = l["level"].as<std::string>();
            if (l["file"]) logging.file = l["file"].as<std::string>();
            if (l["journald"]) logging.journald = l["journald"].as<bool>();
        }
        
        return true;
        
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parse error: " << e.what() << std::endl;
        return false;
    }
}

void Config::load_from_env() {
    // 环境变量覆盖
    if (const char* val = std::getenv("JWT_SECRET")) {
        auth.jwt_secret = val;
    }
    
    if (const char* val = std::getenv("DATA_PLANE_PORT")) {
        server.port = static_cast<uint16_t>(std::stoi(val));
    }
    
    if (const char* val = std::getenv("DATA_PLANE_HOST")) {
        server.host = val;
    }
    
    if (const char* val = std::getenv("ROS_DOMAIN_ID")) {
        ros2.domain_id = std::stoi(val);
    }
    
    if (const char* val = std::getenv("WATCHDOG_TIMEOUT_MS")) {
        watchdog.timeout_ms = std::stoi(val);
    }

    if (const char* val = std::getenv("CONTROL_PLANE_URL")) {
        control_sync.control_plane_url = val;
    }

    if (const char* val = std::getenv("CONTROL_PLANE_INTERNAL_TOKEN")) {
        control_sync.internal_token = val;
    }
}

std::string Config::expand_env(const std::string& value) {
    // 匹配 ${VAR} 或 $VAR
    std::regex env_regex(R"(\$\{([^}]+)\}|\$([A-Za-z_][A-Za-z0-9_]*))");
    
    std::string result = value;
    std::smatch match;
    
    while (std::regex_search(result, match, env_regex)) {
        std::string var_name = match[1].matched ? match[1].str() : match[2].str();
        std::string replacement;
        
        if (const char* val = std::getenv(var_name.c_str())) {
            replacement = val;
        }
        
        result = result.substr(0, match.position()) + 
                 replacement + 
                 result.substr(match.position() + match.length());
    }
    
    return result;
}

} // namespace qyh::dataplane
