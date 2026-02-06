/**
 * @file config.cpp
 * @brief 媒体平面配置实现
 */

#include "media_plane/config.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <regex>

namespace qyh::mediaplane {

static std::string expand_env(const std::string& value);

bool Config::load_from_file(const std::string& path) {
    try {
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "Failed to open config file: " << path << std::endl;
            return false;
        }
        
        YAML::Node config = YAML::LoadFile(path);
        
        // Server 配置
        if (config["server"]) {
            auto s = config["server"];
            if (s["host"]) server.host = s["host"].as<std::string>();
            if (s["signaling_port"]) server.signaling_port = s["signaling_port"].as<uint16_t>();
            if (s["max_connections"]) server.max_connections = s["max_connections"].as<size_t>();
            if (s["max_message_bytes"]) server.max_message_bytes = s["max_message_bytes"].as<size_t>();
            if (s["auth_timeout_sec"]) server.auth_timeout_sec = s["auth_timeout_sec"].as<int>();
            if (s["jwt_secret"]) server.jwt_secret = expand_env(s["jwt_secret"].as<std::string>());
            if (s["require_auth"]) server.require_auth = s["require_auth"].as<bool>();
            if (s["auth_audience"]) server.auth_audience = s["auth_audience"].as<std::string>();
            if (s["auth_issuer"]) server.auth_issuer = s["auth_issuer"].as<std::string>();
            if (s["auth_scope"]) server.auth_scope = s["auth_scope"].as<std::string>();
        }
        
        // Video Sources 配置
        if (config["video"]) {
            auto v = config["video"];
            
            // 默认源
            if (v["default_source"]) {
                video.default_source = v["default_source"].as<std::string>();
            }
            
            // 视频源列表
            if (v["sources"]) {
                video.sources.clear();
                video_sources.clear();
                for (const auto& src : v["sources"]) {
                    VideoSourceConfig vs;
                    vs.name = src["name"].as<std::string>();
                    vs.device = src["device"].as<std::string>("");
                    vs.topic = src["topic"].as<std::string>("");   // ROS2 话题
                    vs.type = src["type"].as<std::string>("v4l2");
                    vs.enabled = src["enabled"].as<bool>(true);
                    video.sources.push_back(vs);
                    video_sources.push_back(vs);  // 兼容旧代码
                }
            }
            
            // Encoding 配置
            if (v["encoding"]) {
                auto e = v["encoding"];
                if (e["codec"]) encoding.codec = e["codec"].as<std::string>();
                if (e["hardware_encoder"]) encoding.hardware_encoder = e["hardware_encoder"].as<bool>();
                if (e["width"]) encoding.width = e["width"].as<int>();
                if (e["height"]) encoding.height = e["height"].as<int>();
                if (e["framerate"]) encoding.framerate = e["framerate"].as<int>();
                if (e["bitrate"]) encoding.bitrate = e["bitrate"].as<int>();
                if (e["keyframe_interval"]) encoding.keyframe_interval = e["keyframe_interval"].as<int>();
            }
        }
        
        // ROS2 配置
        if (config["ros2"]) {
            auto r = config["ros2"];
            if (r["enabled"]) ros2.enabled = r["enabled"].as<bool>();
            if (r["domain_id"]) ros2.domain_id = r["domain_id"].as<int>();
            if (r["discovery_interval"]) ros2.discovery_interval = r["discovery_interval"].as<int>();
        }
        
        // WebRTC 配置
        if (config["webrtc"]) {
            auto w = config["webrtc"];
            if (w["stun_servers"]) {
                webrtc.stun_servers.clear();
                for (const auto& s : w["stun_servers"]) {
                    webrtc.stun_servers.push_back(s.as<std::string>());
                }
            }
            if (w["turn_servers"]) {
                webrtc.turn_servers.clear();
                for (const auto& s : w["turn_servers"]) {
                    webrtc.turn_servers.push_back(s.as<std::string>());
                }
            }
            if (w["ice_transport_policy"]) {
                webrtc.ice_transport_policy = w["ice_transport_policy"].as<std::string>();
            }
        }
        
        // Jetson 配置
        if (config["jetson"]) {
            auto j = config["jetson"];
            if (j["use_nvenc"]) jetson.use_nvenc = j["use_nvenc"].as<bool>();
            if (j["nvenc_preset"]) jetson.nvenc_preset = j["nvenc_preset"].as<std::string>();
            if (j["use_nvdec"]) jetson.use_nvdec = j["use_nvdec"].as<bool>();
        }
        
        // Logging 配置
        if (config["logging"]) {
            auto l = config["logging"];
            if (l["level"]) logging.level = l["level"].as<std::string>();
            if (l["file"]) logging.file = l["file"].as<std::string>();
            if (l["journald"]) logging.journald = l["journald"].as<bool>();
        }
        
        std::cout << "Loaded config from: " << path << std::endl;
        std::cout << "  - Signaling port: " << server.signaling_port << std::endl;
        std::cout << "  - Video sources: " << video.sources.size() << std::endl;
        std::cout << "  - Auth required: " << (server.require_auth ? "yes" : "no") << std::endl;
        
        return true;
        
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parse error: " << e.what() << std::endl;
        return false;
    }
}

void Config::load_from_env() {
    // 环境变量覆盖
    if (const char* val = std::getenv("MEDIA_PLANE_PORT")) {
        server.signaling_port = static_cast<uint16_t>(std::stoi(val));
    }
    
    if (const char* val = std::getenv("MEDIA_PLANE_HOST")) {
        server.host = val;
    }

    if (const char* val = std::getenv("MEDIA_PLANE_MAX_MESSAGE_BYTES")) {
        server.max_message_bytes = static_cast<size_t>(std::stoull(val));
    }

    if (const char* val = std::getenv("MEDIA_PLANE_AUTH_TIMEOUT_SEC")) {
        server.auth_timeout_sec = std::stoi(val);
    }
    
    if (const char* val = std::getenv("VIDEO_BITRATE")) {
        encoding.bitrate = std::stoi(val);
    }
    
    if (const char* val = std::getenv("VIDEO_WIDTH")) {
        encoding.width = std::stoi(val);
    }
    
    if (const char* val = std::getenv("VIDEO_HEIGHT")) {
        encoding.height = std::stoi(val);
    }
    
    if (const char* val = std::getenv("USE_NVENC")) {
        jetson.use_nvenc = (std::string(val) == "true" || std::string(val) == "1");
    }
    
    // JWT 密钥从环境变量获取（安全性更高）
    if (const char* val = std::getenv("JWT_SECRET")) {
        server.jwt_secret = val;
    }
    
    if (const char* val = std::getenv("REQUIRE_AUTH")) {
        server.require_auth = (std::string(val) == "true" || std::string(val) == "1");
    }

    if (const char* val = std::getenv("MEDIA_PLANE_AUTH_AUDIENCE")) {
        server.auth_audience = val;
    }

    if (const char* val = std::getenv("MEDIA_PLANE_AUTH_ISSUER")) {
        server.auth_issuer = val;
    }

    if (const char* val = std::getenv("MEDIA_PLANE_AUTH_SCOPE")) {
        server.auth_scope = val;
    }
}

static std::string expand_env(const std::string& value) {
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

} // namespace qyh::mediaplane
