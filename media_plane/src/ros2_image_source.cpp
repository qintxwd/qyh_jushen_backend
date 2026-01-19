/**
 * @file ros2_image_source.cpp
 * @brief ROS2 图像话题订阅源实现
 */

#include "media_plane/ros2_image_source.hpp"

#include <gst/app/gstappsrc.h>

#include <iostream>
#include <cstring>

namespace qyh::mediaplane {

// ==================== ROS2ImageSource ====================

ROS2ImageSource::ROS2ImageSource(const ROS2ImageSourceConfig& config)
    : config_(config)
{
}

ROS2ImageSource::~ROS2ImageSource() {
    stop();
    
    if (appsrc_) {
        gst_object_unref(appsrc_);
        appsrc_ = nullptr;
    }
    
    subscription_.reset();
    
    if (owns_node_ && node_) {
        node_.reset();
    }
}

bool ROS2ImageSource::init(const std::string& node_name) {
    // 创建自己的节点
    std::string name = node_name.empty() ? 
        "media_plane_ros2_source_" + std::to_string(reinterpret_cast<uintptr_t>(this)) : 
        node_name;
    
    try {
        node_ = std::make_shared<rclcpp::Node>(name);
        owns_node_ = true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to create ROS2 node: " << e.what() << std::endl;
        return false;
    }
    
    return init(node_);
}

bool ROS2ImageSource::init(rclcpp::Node::SharedPtr node) {
    node_ = node;
    
    // 创建 appsrc 元素
    appsrc_ = gst_element_factory_make("appsrc", nullptr);
    if (!appsrc_) {
        std::cerr << "Failed to create appsrc element" << std::endl;
        return false;
    }
    
    // 配置 appsrc
    g_object_set(G_OBJECT(appsrc_),
                 "stream-type", GST_APP_STREAM_TYPE_STREAM,
                 "format", GST_FORMAT_TIME,
                 "is-live", TRUE,
                 "do-timestamp", TRUE,
                 nullptr);
    
    // 创建订阅
    auto qos = rclcpp::QoS(config_.queue_size)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
        config_.topic_name,
        qos,
        std::bind(&ROS2ImageSource::image_callback, this, std::placeholders::_1)
    );
    
    std::cout << "ROS2 image source initialized for topic: " << config_.topic_name << std::endl;
    
    return true;
}

bool ROS2ImageSource::start() {
    if (running_) {
        return true;
    }
    
    running_ = true;
    start_time_ = std::chrono::steady_clock::now();
    
    // 如果是自己创建的节点，需要启动 spin 线程
    if (owns_node_ && node_) {
        spin_thread_ = std::thread([this]() {
            while (running_ && rclcpp::ok()) {
                rclcpp::spin_some(node_);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }
    
    std::cout << "ROS2 image source started" << std::endl;
    return true;
}

void ROS2ImageSource::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    
    // 发送 EOS
    if (appsrc_) {
        gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
    }
    
    std::cout << "ROS2 image source stopped" << std::endl;
}

double ROS2ImageSource::get_fps() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
    if (duration > 0) {
        return static_cast<double>(frame_count_) / duration;
    }
    return 0.0;
}

void ROS2ImageSource::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!running_ || !appsrc_) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 首次收到图像，设置 caps
    if (!caps_set_) {
        width_ = msg->width;
        height_ = msg->height;
        encoding_ = msg->encoding;
        
        std::string gst_format = get_gst_format(encoding_);
        if (gst_format.empty()) {
            std::cerr << "Unsupported image encoding: " << encoding_ << std::endl;
            return;
        }
        
        // 设置 caps
        GstCaps* caps = gst_caps_new_simple(
            "video/x-raw",
            "format", G_TYPE_STRING, gst_format.c_str(),
            "width", G_TYPE_INT, width_,
            "height", G_TYPE_INT, height_,
            "framerate", GST_TYPE_FRACTION, 30, 1,  // 假设 30fps
            nullptr
        );
        
        gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
        gst_caps_unref(caps);
        
        caps_set_ = true;
        connected_ = true;
        
        std::cout << "ROS2 image source connected: " << width_ << "x" << height_ 
                  << " " << encoding_ << " -> " << gst_format << std::endl;
        
        if (frame_callback_) {
            frame_callback_(width_, height_, encoding_);
        }
    }
    
    // 转换并推送
    convert_and_push(msg);
    
    frame_count_++;
}

bool ROS2ImageSource::convert_and_push(const sensor_msgs::msg::Image::SharedPtr& msg) {
    // 计算数据大小
    size_t data_size = msg->data.size();
    
    // 创建 GstBuffer
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, data_size, nullptr);
    if (!buffer) {
        std::cerr << "Failed to allocate GstBuffer" << std::endl;
        return false;
    }
    
    // 复制数据
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        std::memcpy(map.data, msg->data.data(), data_size);
        gst_buffer_unmap(buffer, &map);
    } else {
        gst_buffer_unref(buffer);
        return false;
    }
    
    // 设置时间戳
    GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;
    GST_BUFFER_DTS(buffer) = GST_CLOCK_TIME_NONE;
    
    // 推送到 appsrc
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    if (ret != GST_FLOW_OK) {
        // buffer 已经被 push_buffer 接管，不需要 unref
        return false;
    }
    
    return true;
}

std::string ROS2ImageSource::get_gst_format(const std::string& ros_encoding) {
    // ROS2 sensor_msgs/Image encoding -> GStreamer format
    if (ros_encoding == "rgb8") {
        return "RGB";
    } else if (ros_encoding == "bgr8") {
        return "BGR";
    } else if (ros_encoding == "rgba8") {
        return "RGBA";
    } else if (ros_encoding == "bgra8") {
        return "BGRA";
    } else if (ros_encoding == "mono8") {
        return "GRAY8";
    } else if (ros_encoding == "mono16") {
        return "GRAY16_LE";
    } else if (ros_encoding == "yuv422" || ros_encoding == "yuyv") {
        return "YUY2";
    } else if (ros_encoding == "uyvy") {
        return "UYVY";
    } else if (ros_encoding == "nv12") {
        return "NV12";
    } else if (ros_encoding == "nv21") {
        return "NV21";
    }
    
    // Orbbec 摄像头通常使用 rgb8 或 bgr8
    std::cerr << "Unknown ROS encoding: " << ros_encoding << ", trying BGR" << std::endl;
    return "BGR";
}

// ==================== ROS2ImageSourceFactory ====================

ROS2ImageSourceFactory& ROS2ImageSourceFactory::instance() {
    static ROS2ImageSourceFactory instance;
    return instance;
}

void ROS2ImageSourceFactory::init_ros2(int argc, char** argv) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (ros2_initialized_) {
        return;
    }
    
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }
    
    // 创建共享节点
    shared_node_ = std::make_shared<rclcpp::Node>("media_plane_ros2");
    
    // 创建 executor
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(shared_node_);
    
    // 启动 executor 线程
    executor_thread_ = std::thread([this]() {
        executor_->spin();
    });
    
    ros2_initialized_ = true;
    std::cout << "ROS2 initialized for Media Plane" << std::endl;
}

void ROS2ImageSourceFactory::shutdown_ros2() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!ros2_initialized_) {
        return;
    }
    
    if (executor_) {
        executor_->cancel();
    }
    
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    
    executor_.reset();
    shared_node_.reset();
    
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    
    ros2_initialized_ = false;
    std::cout << "ROS2 shutdown for Media Plane" << std::endl;
}

rclcpp::Node::SharedPtr ROS2ImageSourceFactory::get_shared_node() {
    std::lock_guard<std::mutex> lock(mutex_);
    return shared_node_;
}

std::shared_ptr<ROS2ImageSource> ROS2ImageSourceFactory::create_source(const std::string& topic) {
    ROS2ImageSourceConfig config;
    config.topic_name = topic;
    config.queue_size = 1;
    return create_source(config);
}

std::shared_ptr<ROS2ImageSource> ROS2ImageSourceFactory::create_source(
    const ROS2ImageSourceConfig& config) {
    auto source = std::make_shared<ROS2ImageSource>(config);

    auto node = get_shared_node();
    if (node) {
        source->init(node);
    } else {
        source->init();
    }

    return source;
}

std::vector<std::string> ROS2ImageSourceFactory::discover_camera_topics() {
    std::vector<std::string> camera_topics;
    
    auto node = get_shared_node();
    if (!node) {
        return camera_topics;
    }
    
    // 获取所有话题
    auto topic_names_and_types = node->get_topic_names_and_types();
    
    for (const auto& [topic, types] : topic_names_and_types) {
        for (const auto& type : types) {
            if (type == "sensor_msgs/msg/Image") {
                // 过滤只包含 color/image 的话题
                if (topic.find("/color/image") != std::string::npos ||
                    topic.find("/rgb/image") != std::string::npos ||
                    topic.find("/image_raw") != std::string::npos) {
                    camera_topics.push_back(topic);
                }
            }
        }
    }
    
    return camera_topics;
}

} // namespace qyh::mediaplane
