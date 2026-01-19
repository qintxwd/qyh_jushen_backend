/**
 * @file main.cpp
 * @brief Data Plane 主程序入口
 */

#include "data_plane/server.hpp"
#include "data_plane/config.hpp"
#include "data_plane/auth.hpp"
#include "data_plane/message_handler.hpp"
#include "data_plane/state_cache.hpp"

#ifdef WITH_ROS2
#include "data_plane/ros2_bridge.hpp"
#include "data_plane/watchdog.hpp"
#include <rclcpp/rclcpp.hpp>
#endif

#include <boost/asio/io_context.hpp>
#include <boost/asio/signal_set.hpp>

#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace net = boost::asio;

int main(int argc, char* argv[]) {
    // 配置文件路径
    std::string config_path = "/etc/qyh-robot/data_plane/config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }
    
    // 加载配置
    qyh::dataplane::Config config;
    if (!config.load_from_file(config_path)) {
        std::cerr << "Failed to load config from: " << config_path << std::endl;
        std::cerr << "Using default configuration" << std::endl;
    }
    config.load_from_env();
    
    std::cout << "=== QYH Robot Data Plane ===" << std::endl;
    std::cout << "Version: 2.0.0" << std::endl;
    std::cout << "Config: " << config_path << std::endl;
    std::cout << "Listen: " << config.server.host << ":" << config.server.port << std::endl;
    
#ifdef WITH_ROS2
    std::cout << "ROS2: Enabled" << std::endl;
    
    // 初始化 ROS2
    rclcpp::init(argc, argv);
#else
    std::cout << "ROS2: Disabled" << std::endl;
#endif
    
    try {
        // 创建 IO 上下文
        net::io_context io_context;
        
        // 创建组件
        qyh::dataplane::StateCache state_cache;
        qyh::dataplane::JWTValidator validator(config.auth.jwt_secret, 
                                                config.auth.jwt_algorithm);
        qyh::dataplane::MessageHandler handler(config, validator, state_cache);
        
#ifdef WITH_ROS2
        // 创建 ROS2 桥接
        qyh::dataplane::ROS2Bridge ros2_bridge(config, state_cache);
        if (!ros2_bridge.init()) {
            std::cerr << "Failed to initialize ROS2 bridge" << std::endl;
            return 1;
        }
        
        // 创建 Watchdog
        qyh::dataplane::Watchdog watchdog(config, ros2_bridge);
        
        // 连接组件
        handler.set_ros2_bridge(&ros2_bridge);
        handler.set_watchdog(&watchdog);
#endif
        
        // 创建服务器
        auto server = std::make_shared<qyh::dataplane::Server>(io_context, config);
        
#ifdef WITH_ROS2
        ros2_bridge.set_server(server.get());
        
        // 启动 ROS2 桥接
        ros2_bridge.start();
        
        // 启动 Watchdog
        watchdog.start();
#endif
        
        // 启动服务器
        server->start();
        
        // 信号处理
        net::signal_set signals(io_context, SIGINT, SIGTERM);
        signals.async_wait([&](const boost::system::error_code&, int signal) {
            std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
            server->stop();
#ifdef WITH_ROS2
            watchdog.stop();
            ros2_bridge.stop();
#endif
            io_context.stop();
        });
        
        // 运行 IO 上下文（多线程）
        const auto thread_count = std::max(1u, std::thread::hardware_concurrency());
        std::vector<std::thread> threads;
        threads.reserve(thread_count - 1);
        
        for (auto i = 0u; i < thread_count - 1; ++i) {
            threads.emplace_back([&io_context]() {
                io_context.run();
            });
        }
        
        // 主线程也运行 IO
        io_context.run();
        
        // 等待所有线程结束
        for (auto& t : threads) {
            if (t.joinable()) {
                t.join();
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
#ifdef WITH_ROS2
    rclcpp::shutdown();
#endif
    
    std::cout << "Data Plane shutdown complete" << std::endl;
    return 0;
}
