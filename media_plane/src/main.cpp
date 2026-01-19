/**
 * @file main.cpp
 * @brief Media Plane 主程序入口
 */

#include "media_plane/media_server.hpp"
#include "media_plane/config.hpp"

#include <gst/gst.h>

#include <iostream>
#include <csignal>
#include <atomic>

static std::atomic<bool> g_running{true};

void signal_handler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    g_running = false;
}

int main(int argc, char* argv[]) {
    // 配置文件路径
    std::string config_path = "/etc/qyh-robot/media_plane/config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }
    
    // 初始化 GStreamer
    gst_init(&argc, &argv);
    
    // 加载配置
    qyh::mediaplane::Config config;
    if (!config.load_from_file(config_path)) {
        std::cerr << "Failed to load config from: " << config_path << std::endl;
        std::cerr << "Using default configuration" << std::endl;
    }
    config.load_from_env();
    
    std::cout << "=== QYH Robot Media Plane ===" << std::endl;
    std::cout << "Version: 2.0.0" << std::endl;
    std::cout << "Config: " << config_path << std::endl;
    std::cout << "Signaling Port: " << config.server.signaling_port << std::endl;
    std::cout << "Video Codec: " << config.encoding.codec << std::endl;
    std::cout << "Resolution: " << config.encoding.width << "x" << config.encoding.height << std::endl;
    std::cout << "Hardware Encoder: " << (config.jetson.use_nvenc ? "Yes (NVENC)" : "No") << std::endl;
    
    // 信号处理
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    try {
        // 创建媒体服务器
        qyh::mediaplane::MediaServer server(config);
        
        if (!server.init()) {
            std::cerr << "Failed to initialize media server" << std::endl;
            return 1;
        }
        
        // 启动服务器
        server.start();
        
        // 主循环
        while (g_running && server.is_running()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 停止服务器
        server.stop();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    // 清理 GStreamer
    gst_deinit();
    
    std::cout << "Media Plane shutdown complete" << std::endl;
    return 0;
}
