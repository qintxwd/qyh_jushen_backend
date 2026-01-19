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

namespace net = boost::asio;

MediaServer::MediaServer(const Config& config)
    : config_(config)
{
}

MediaServer::~MediaServer() {
    stop();
}

bool MediaServer::init() {
    // 创建 IO Context
    io_context_ = std::make_unique<net::io_context>();
    
    // 创建管道管理器
    pipeline_manager_ = std::make_unique<PipelineManager>(config_);
    if (!pipeline_manager_->init()) {
        std::cerr << "Failed to initialize pipeline manager" << std::endl;
        return false;
    }
    
    // 创建信令服务器
    signaling_server_ = std::make_shared<SignalingServer>(*io_context_, config_);
    
    // 互相关联
    signaling_server_->set_pipeline_manager(pipeline_manager_.get());
    pipeline_manager_->set_signaling_server(signaling_server_.get());
    
    std::cout << "Media server initialized" << std::endl;
    return true;
}

void MediaServer::start() {
    if (running_) {
        return;
    }
    
    running_ = true;
    
    // 启动信令服务器
    signaling_server_->start();
    std::cout << "Signaling server started on port " << config_.server.signaling_port << std::endl;
    
    // 启动 IO 线程
    io_thread_ = std::thread([this]() {
        try {
            io_context_->run();
        } catch (const std::exception& e) {
            std::cerr << "IO context error: " << e.what() << std::endl;
        }
    });
    
    // 启动视频源
    if (!pipeline_manager_->start_sources()) {
        std::cerr << "Failed to start video sources" << std::endl;
    }
    
    std::cout << "Media server started" << std::endl;
}

void MediaServer::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    // 停止信令服务器
    if (signaling_server_) {
        signaling_server_->stop();
    }
    
    // 停止视频源
    if (pipeline_manager_) {
        pipeline_manager_->stop_sources();
    }
    
    // 停止 IO Context
    if (io_context_) {
        io_context_->stop();
    }
    
    // 等待 IO 线程结束
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    
    std::cout << "Media server stopped" << std::endl;
}

} // namespace qyh::mediaplane
