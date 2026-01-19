/**
 * @file media_server.hpp
 * @brief 媒体服务器主类
 */

#pragma once

#include <memory>
#include <atomic>
#include <string>
#include <thread>

namespace boost::asio {
    class io_context;
}

namespace qyh::mediaplane {

// 前向声明
class Config;
class SignalingServer;
class PipelineManager;

/**
 * @brief 媒体服务器
 * 
 * 协调 WebRTC 信令和 GStreamer 管道
 */
class MediaServer {
public:
    /**
     * @brief 构造函数
     * @param config 配置
     */
    explicit MediaServer(const Config& config);
    
    ~MediaServer();
    
    /**
     * @brief 初始化服务器
     * @return 是否成功
     */
    bool init();
    
    /**
     * @brief 启动服务器
     */
    void start();
    
    /**
     * @brief 停止服务器
     */
    void stop();
    
    /**
     * @brief 是否正在运行
     */
    bool is_running() const { return running_; }
    
private:
    const Config& config_;
    
    std::unique_ptr<boost::asio::io_context> io_context_;
    std::shared_ptr<SignalingServer> signaling_server_;
    std::unique_ptr<PipelineManager> pipeline_manager_;
    
    std::thread io_thread_;
    std::atomic<bool> running_{false};
};

} // namespace qyh::mediaplane
