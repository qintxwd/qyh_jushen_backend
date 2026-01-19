/**
 * @file ros2_image_source.hpp
 * @brief ROS2 图像话题订阅源
 * 
 * 订阅 ROS2 sensor_msgs/Image 话题，转换为 GStreamer buffer
 * 用于支持 Orbbec RGBD 摄像头等通过 ROS2 发布图像的设备
 */

#pragma once

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <functional>

namespace qyh::mediaplane {

/**
 * @brief ROS2 图像源配置
 */
struct ROS2ImageSourceConfig {
    std::string topic_name;           // ROS2 话题名称，如 "/head_camera/color/image_raw"
    std::string frame_id;             // 可选的 frame_id 过滤
    int queue_size = 1;               // 订阅队列大小（1=最新帧）
    bool use_compressed = false;      // 是否使用压缩话题
};

/**
 * @brief ROS2 图像话题订阅者
 * 
 * 订阅 ROS2 图像话题，将 sensor_msgs/Image 转换为 GStreamer buffer
 * 通过 appsrc 元素注入到 GStreamer pipeline
 */
class ROS2ImageSource {
public:
    /**
     * @brief 构造函数
     * @param config 配置
     */
    explicit ROS2ImageSource(const ROS2ImageSourceConfig& config);
    
    ~ROS2ImageSource();
    
    /**
     * @brief 初始化 ROS2 节点和订阅
     * @param node_name 节点名称（可选，为空则使用共享节点）
     * @return 是否成功
     */
    bool init(const std::string& node_name = "");
    
    /**
     * @brief 使用已有的 ROS2 节点
     * @param node 已存在的节点
     * @return 是否成功
     */
    bool init(rclcpp::Node::SharedPtr node);
    
    /**
     * @brief 启动订阅
     */
    bool start();
    
    /**
     * @brief 停止订阅
     */
    void stop();
    
    /**
     * @brief 获取 appsrc 元素（用于连接到 GStreamer pipeline）
     */
    GstElement* get_appsrc() const { return appsrc_; }
    
    /**
     * @brief 获取图像信息
     */
    int get_width() const { return width_; }
    int get_height() const { return height_; }
    std::string get_encoding() const { return encoding_; }
    
    /**
     * @brief 是否已连接（收到过图像）
     */
    bool is_connected() const { return connected_; }
    
    /**
     * @brief 获取帧率统计
     */
    double get_fps() const;
    
    /**
     * @brief 设置帧回调（用于调试）
     */
    using FrameCallback = std::function<void(int width, int height, const std::string& encoding)>;
    void set_frame_callback(FrameCallback callback) { frame_callback_ = std::move(callback); }
    
private:
    /**
     * @brief 图像回调
     */
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief 转换图像格式
     */
    bool convert_and_push(const sensor_msgs::msg::Image::SharedPtr& msg);
    
    /**
     * @brief 获取 GStreamer caps 格式
     */
    std::string get_gst_format(const std::string& ros_encoding);
    
private:
    ROS2ImageSourceConfig config_;
    
    // ROS2
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    bool owns_node_ = false;
    std::thread spin_thread_;
    std::atomic<bool> running_{false};
    
    // GStreamer
    GstElement* appsrc_ = nullptr;
    
    // 图像信息
    int width_ = 0;
    int height_ = 0;
    std::string encoding_;
    std::atomic<bool> connected_{false};
    bool caps_set_ = false;
    
    // 统计
    std::atomic<uint64_t> frame_count_{0};
    std::chrono::steady_clock::time_point start_time_;
    
    // 回调
    FrameCallback frame_callback_;
    
    std::mutex mutex_;
};

/**
 * @brief ROS2 图像源工厂
 * 
 * 管理多个 ROS2 图像源，提供统一的 ROS2 节点
 */
class ROS2ImageSourceFactory {
public:
    /**
     * @brief 获取单例
     */
    static ROS2ImageSourceFactory& instance();
    
    /**
     * @brief 初始化 ROS2（如果还没初始化）
     * @param argc 命令行参数数量
     * @param argv 命令行参数
     */
    void init_ros2(int argc = 0, char** argv = nullptr);
    
    /**
     * @brief 关闭 ROS2
     */
    void shutdown_ros2();
    
    /**
     * @brief 获取共享节点
     */
    rclcpp::Node::SharedPtr get_shared_node();
    
    /**
     * @brief 创建图像源
     * @param topic 话题名称
     * @return 图像源
     */
    std::unique_ptr<ROS2ImageSource> create_source(const std::string& topic);
    
    /**
     * @brief 检测可用的摄像头话题
     * @return 话题列表
     */
    std::vector<std::string> discover_camera_topics();
    
private:
    ROS2ImageSourceFactory() = default;
    
    rclcpp::Node::SharedPtr shared_node_;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    std::thread executor_thread_;
    std::atomic<bool> ros2_initialized_{false};
    std::mutex mutex_;
};

} // namespace qyh::mediaplane
