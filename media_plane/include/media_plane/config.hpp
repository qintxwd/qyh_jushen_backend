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
    std::string name;           // 源名称（如 "head_camera"）
    std::string device;         // 设备路径（如 "/dev/video0"）用于 v4l2
    std::string topic;          // ROS2 话题（如 "/head_camera/color/image_raw"）用于 ros2 类型
    std::string type;           // 源类型（ros2/v4l2/nvargus/test）
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
    size_t max_message_bytes = 1024 * 1024;
    int auth_timeout_sec = 10;
    
    // JWT 认证配置
    std::string jwt_secret;     // JWT 密钥（与 Control Plane 共享）
    bool require_auth = false;  // 是否要求认证（开发模式可关闭）
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
 * @brief 视频配置（包含多个视频源）
 */
struct VideoConfig {
    std::vector<VideoSourceConfig> sources;
    std::string default_source = "head_camera";
};

/**
 * @brief ROS2 配置
 */
struct ROS2Config {
    bool enabled = true;
    int domain_id = -1;         // -1 表示使用环境变量
    int discovery_interval = 5;  // 话题发现间隔（秒）
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
    VideoConfig video;
    std::vector<VideoSourceConfig> video_sources;  // 兼容旧代码
    EncodingConfig encoding;
    WebRTCConfig webrtc;
    JetsonConfig jetson;
    ROS2Config ros2;
    LoggingConfig logging;
};

} // namespace qyh::mediaplane
