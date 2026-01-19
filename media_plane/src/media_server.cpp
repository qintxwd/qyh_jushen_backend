/**
 * @file media_server.cpp
 * @brief 媒体服务器实现
 */

#include "media_plane/media_server.hpp"
#include "media_plane/config.hpp"
#include "media_plane/signaling_server.hpp"
#include "media_plane/pipeline_manager.hpp"

#include <boost/asio/io_context.hpp>

#include <iostream>
#include <thread>

namespace qyh::mediaplane {

class MediaServerImpl {
public:
    MediaServerImpl(const Config& config)
        : config_(config)
        , io_context_()
    {}
    
    const Config& config_;
    net::io_context io_context_;
    std::thread io_thread_;
};

MediaServer::MediaServer(const Config& config)
    : config_(config)
{
}

MediaServer::~MediaServer() {
    stop();
}

bool MediaServer::init() {
    // 创建管道管理器
    pipeline_manager_ = std::make_unique<PipelineManager>(config_);
    if (!pipeline_manager_->init()) {
        std::cerr << "Failed to initialize pipeline manager" << std::endl;
        return false;
    }
    
    std::cout << "Media server initialized" << std::endl;
    return true;
}

void MediaServer::start() {
    if (running_) {
        return;
    }
    
    running_ = true;
    
    // 启动视频源
    if (!pipeline_manager_->start_sources()) {
        std::cerr << "Failed to start video sources" << std::endl;
    }
    
    // TODO: 启动信令服务器
    // 需要在单独线程运行 io_context
    
    std::cout << "Media server started" << std::endl;
}

void MediaServer::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    // 停止视频源
    if (pipeline_manager_) {
        pipeline_manager_->stop_sources();
    }
    
    // TODO: 停止信令服务器
    
    std::cout << "Media server stopped" << std::endl;
}

} // namespace qyh::mediaplane
