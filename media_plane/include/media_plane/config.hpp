/**
 * @file config.hpp
 * @brief 媒体平面配置
 */

#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace qyh::mediaplane {

/**
 * @brief 视频源配置
 */
struct VideoSourceConfig {
    std::string name;
    std::string device;
    std::string type;
    bool enabled = true;
};

/**
 * @brief 编码配置
 */
struct EncodingConfig {
    std::string codec = "h264";
    bool hardware_encoder = true;
    int width = 1280;
    int height = 720;
    int framerate = 30;
    int bitrate = 2000;
    int keyframe_interval = 30;
};

/**
 * @brief WebRTC 配置
 */
struct WebRTCConfig {
    std::vector<std::string> stun_servers;
    std::vector<std::string> turn_servers;
    std::string ice_transport_policy = "all";
};

/**
 * @brief Jetson 配置
 */
struct JetsonConfig {
    bool use_nvenc = true;
    std::string nvenc_preset = "fast";
    bool use_nvdec = false;
};

/**
 * @brief 服务器配置
 */
struct ServerConfig {
    std::string host = "0.0.0.0";
    uint16_t signaling_port = 8888;
    size_t max_connections = 10;
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
    
    // 配置项
    ServerConfig server;
    std::vector<VideoSourceConfig> video_sources;
    EncodingConfig encoding;
    WebRTCConfig webrtc;
    JetsonConfig jetson;
    LoggingConfig logging;
};

} // namespace qyh::mediaplane
