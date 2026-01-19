/**
 * @file imu_tf_led.hpp
 * @brief IMU/TF/LED 支持模块
 * 
 * 支持：
 * - IMU 数据订阅和转发
 * - TF 坐标变换数据订阅
 * - LED 控制命令发布
 */

#pragma once

#include <string>
#include <functional>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <chrono>
#include <cmath>
#include <array>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/int32.hpp>

namespace qyh::dataplane {

// ==================== IMU 数据 ====================

/**
 * @brief IMU 数据结构
 */
struct ImuData {
    struct {
        double x = 0, y = 0, z = 0, w = 1;
    } orientation;
    
    struct {
        double x = 0, y = 0, z = 0;
    } angular_velocity;
    
    struct {
        double x = 0, y = 0, z = 0;
    } linear_acceleration;
    
    // 协方差（可选）
    std::array<double, 9> orientation_covariance{};
    std::array<double, 9> angular_velocity_covariance{};
    std::array<double, 9> linear_acceleration_covariance{};
    
    // 时间戳
    double timestamp_sec = 0;
    
    // 帧 ID
    std::string frame_id;
    
    /**
     * @brief 转换为 JSON
     */
    std::string to_json() const {
        char buf[1024];
        snprintf(buf, sizeof(buf), R"({
  "orientation": {"x": %.6f, "y": %.6f, "z": %.6f, "w": %.6f},
  "angular_velocity": {"x": %.6f, "y": %.6f, "z": %.6f},
  "linear_acceleration": {"x": %.6f, "y": %.6f, "z": %.6f},
  "timestamp": %.6f,
  "frame_id": "%s"
})",
            orientation.x, orientation.y, orientation.z, orientation.w,
            angular_velocity.x, angular_velocity.y, angular_velocity.z,
            linear_acceleration.x, linear_acceleration.y, linear_acceleration.z,
            timestamp_sec, frame_id.c_str());
        return buf;
    }
    
    /**
     * @brief 计算欧拉角（roll, pitch, yaw）
     */
    void get_euler(double& roll, double& pitch, double& yaw) const {
        // 四元数转欧拉角
        double sinr_cosp = 2 * (orientation.w * orientation.x + orientation.y * orientation.z);
        double cosr_cosp = 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y);
        roll = std::atan2(sinr_cosp, cosr_cosp);
        
        double sinp = 2 * (orientation.w * orientation.y - orientation.z * orientation.x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);
        
        double siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y);
        double cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
};

// ==================== TF 变换 ====================

/**
 * @brief TF 变换数据
 */
struct TfTransform {
    std::string parent_frame;
    std::string child_frame;
    
    struct {
        double x = 0, y = 0, z = 0;
    } translation;
    
    struct {
        double x = 0, y = 0, z = 0, w = 1;
    } rotation;
    
    double timestamp_sec = 0;
    
    /**
     * @brief 转换为 JSON
     */
    std::string to_json() const {
        char buf[512];
        snprintf(buf, sizeof(buf), R"({
  "parent_frame": "%s",
  "child_frame": "%s",
  "translation": {"x": %.6f, "y": %.6f, "z": %.6f},
  "rotation": {"x": %.6f, "y": %.6f, "z": %.6f, "w": %.6f},
  "timestamp": %.6f
})",
            parent_frame.c_str(), child_frame.c_str(),
            translation.x, translation.y, translation.z,
            rotation.x, rotation.y, rotation.z, rotation.w,
            timestamp_sec);
        return buf;
    }
};

/**
 * @brief TF 缓存
 */
class TfCache {
public:
    /**
     * @brief 更新变换
     */
    void update(const TfTransform& tf) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string key = tf.parent_frame + "->" + tf.child_frame;
        transforms_[key] = tf;
    }
    
    /**
     * @brief 获取变换
     */
    bool get(const std::string& parent, const std::string& child, TfTransform& tf) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string key = parent + "->" + child;
        auto it = transforms_.find(key);
        if (it != transforms_.end()) {
            tf = it->second;
            return true;
        }
        return false;
    }
    
    /**
     * @brief 获取所有变换
     */
    std::vector<TfTransform> get_all() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<TfTransform> result;
        for (const auto& [key, tf] : transforms_) {
            result.push_back(tf);
        }
        return result;
    }
    
    /**
     * @brief 导出为 JSON
     */
    std::string to_json() const {
        auto all = get_all();
        std::string result = "[";
        for (size_t i = 0; i < all.size(); ++i) {
            if (i > 0) result += ",";
            result += all[i].to_json();
        }
        result += "]";
        return result;
    }
    
private:
    std::unordered_map<std::string, TfTransform> transforms_;
    mutable std::mutex mutex_;
};

// ==================== LED 控制 ====================

/**
 * @brief LED 模式枚举
 */
enum class LedMode {
    OFF = 0,
    SOLID = 1,      // 常亮
    BLINK = 2,      // 闪烁
    BREATHE = 3,    // 呼吸
    RAINBOW = 4,    // 彩虹
    PULSE = 5,      // 脉冲
    CUSTOM = 99     // 自定义
};

/**
 * @brief LED 命令
 */
struct LedCommand {
    LedMode mode = LedMode::OFF;
    
    // 颜色 (RGB, 0-255)
    uint8_t r = 0, g = 0, b = 0;
    
    // 亮度 (0-100)
    uint8_t brightness = 100;
    
    // 闪烁/呼吸频率 (Hz)
    float frequency = 1.0f;
    
    // 持续时间 (秒，0 表示永久)
    float duration = 0;
    
    // LED 区域 (空表示全部)
    std::string region;
    
    /**
     * @brief 从 JSON 解析
     */
    static LedCommand from_json(const std::string& json);
    
    /**
     * @brief 转换为 JSON
     */
    std::string to_json() const {
        char buf[256];
        snprintf(buf, sizeof(buf), R"({
  "mode": %d,
  "color": {"r": %d, "g": %d, "b": %d},
  "brightness": %d,
  "frequency": %.2f,
  "duration": %.2f,
  "region": "%s"
})",
            static_cast<int>(mode), r, g, b, brightness,
            frequency, duration, region.c_str());
        return buf;
    }
    
    /**
     * @brief 预设：关闭
     */
    static LedCommand off() {
        return LedCommand{};
    }
    
    /**
     * @brief 预设：红色警告
     */
    static LedCommand warning() {
        LedCommand cmd;
        cmd.mode = LedMode::BLINK;
        cmd.r = 255; cmd.g = 0; cmd.b = 0;
        cmd.frequency = 2.0f;
        return cmd;
    }
    
    /**
     * @brief 预设：绿色就绪
     */
    static LedCommand ready() {
        LedCommand cmd;
        cmd.mode = LedMode::SOLID;
        cmd.r = 0; cmd.g = 255; cmd.b = 0;
        return cmd;
    }
    
    /**
     * @brief 预设：蓝色运行中
     */
    static LedCommand running() {
        LedCommand cmd;
        cmd.mode = LedMode::BREATHE;
        cmd.r = 0; cmd.g = 100; cmd.b = 255;
        cmd.frequency = 0.5f;
        return cmd;
    }
    
    /**
     * @brief 预设：充电中
     */
    static LedCommand charging() {
        LedCommand cmd;
        cmd.mode = LedMode::PULSE;
        cmd.r = 255; cmd.g = 165; cmd.b = 0;  // 橙色
        cmd.frequency = 1.0f;
        return cmd;
    }
};

/**
 * @brief LED 控制器
 */
class LedController {
public:
    using CommandCallback = std::function<void(const LedCommand&)>;
    
    /**
     * @brief 设置命令发送回调
     */
    void set_command_callback(CommandCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_callback_ = std::move(callback);
    }
    
    /**
     * @brief 发送 LED 命令
     */
    bool send_command(const LedCommand& cmd) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (command_callback_) {
            command_callback_(cmd);
            last_command_ = cmd;
            return true;
        }
        return false;
    }
    
    /**
     * @brief 获取最后发送的命令
     */
    LedCommand get_last_command() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_command_;
    }
    
    // 便捷方法
    bool off() { return send_command(LedCommand::off()); }
    bool warning() { return send_command(LedCommand::warning()); }
    bool ready() { return send_command(LedCommand::ready()); }
    bool running() { return send_command(LedCommand::running()); }
    bool charging() { return send_command(LedCommand::charging()); }
    
private:
    CommandCallback command_callback_;
    LedCommand last_command_;
    mutable std::mutex mutex_;
};

// ==================== ROS2 适配器 ====================


/**
 * @brief IMU/TF/LED ROS2 适配器
 */
class ImuTfLedRos2Adapter {
public:
    using ImuCallback = std::function<void(const ImuData&)>;
    using TfCallback = std::function<void(const std::vector<TfTransform>&)>;
    
    ImuTfLedRos2Adapter(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        // 订阅 IMU
        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                ImuData data;
                data.orientation.x = msg->orientation.x;
                data.orientation.y = msg->orientation.y;
                data.orientation.z = msg->orientation.z;
                data.orientation.w = msg->orientation.w;
                data.angular_velocity.x = msg->angular_velocity.x;
                data.angular_velocity.y = msg->angular_velocity.y;
                data.angular_velocity.z = msg->angular_velocity.z;
                data.linear_acceleration.x = msg->linear_acceleration.x;
                data.linear_acceleration.y = msg->linear_acceleration.y;
                data.linear_acceleration.z = msg->linear_acceleration.z;
                data.frame_id = msg->header.frame_id;
                data.timestamp_sec = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
                
                std::lock_guard<std::mutex> lock(mutex_);
                last_imu_ = data;
                if (imu_callback_) imu_callback_(data);
            });
        
        // 订阅 TF
        tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 10,
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                std::vector<TfTransform> transforms;
                for (const auto& t : msg->transforms) {
                    TfTransform tf;
                    tf.parent_frame = t.header.frame_id;
                    tf.child_frame = t.child_frame_id;
                    tf.translation.x = t.transform.translation.x;
                    tf.translation.y = t.transform.translation.y;
                    tf.translation.z = t.transform.translation.z;
                    tf.rotation.x = t.transform.rotation.x;
                    tf.rotation.y = t.transform.rotation.y;
                    tf.rotation.z = t.transform.rotation.z;
                    tf.rotation.w = t.transform.rotation.w;
                    tf.timestamp_sec = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9;
                    transforms.push_back(tf);
                    tf_cache_.update(tf);
                }
                
                std::lock_guard<std::mutex> lock(mutex_);
                if (tf_callback_) tf_callback_(transforms);
            });
        
        // 静态 TF
        tf_static_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf_static", rclcpp::QoS(10).transient_local(),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto& t : msg->transforms) {
                    TfTransform tf;
                    tf.parent_frame = t.header.frame_id;
                    tf.child_frame = t.child_frame_id;
                    tf.translation.x = t.transform.translation.x;
                    tf.translation.y = t.transform.translation.y;
                    tf.translation.z = t.transform.translation.z;
                    tf.rotation.x = t.transform.rotation.x;
                    tf.rotation.y = t.transform.rotation.y;
                    tf.rotation.z = t.transform.rotation.z;
                    tf.rotation.w = t.transform.rotation.w;
                    tf.timestamp_sec = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9;
                    tf_cache_.update(tf);
                }
            });
        
        // LED 发布者
        led_pub_ = node_->create_publisher<std_msgs::msg::String>("/led/command", 10);
    }
    
    void set_imu_callback(ImuCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_callback_ = std::move(callback);
    }
    
    void set_tf_callback(TfCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        tf_callback_ = std::move(callback);
    }
    
    ImuData get_last_imu() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_imu_;
    }
    
    TfCache& tf_cache() { return tf_cache_; }
    
    void publish_led_command(const LedCommand& cmd) {
        auto msg = std_msgs::msg::String();
        msg.data = cmd.to_json();
        led_pub_->publish(msg);
    }
    
private:
    rclcpp::Node::SharedPtr node_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr led_pub_;
    
    ImuCallback imu_callback_;
    TfCallback tf_callback_;
    
    ImuData last_imu_;
    TfCache tf_cache_;
    
    mutable std::mutex mutex_;
};

} // namespace qyh::dataplane
