/**
 * @file ros2_bridge.hpp
 * @brief ROS2 桥接模块
 * 
 * 负责与 ROS2 系统通信，订阅状态话题并发布控制命令
 */

#pragma once

#ifdef WITH_ROS2

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <functional>
#include <string>
#include <vector>

namespace qyh::dataplane {

// 前向声明
class Config;
class StateCache;
class Server;

/**
 * @brief ROS2 桥接
 */
class ROS2Bridge {
public:
    /**
     * @brief 构造函数
     * @param config 配置
     * @param state_cache 状态缓存
     */
    ROS2Bridge(const Config& config, StateCache& state_cache);
    
    ~ROS2Bridge();
    
    /**
     * @brief 初始化 ROS2 节点
     * @return 是否成功
     */
    bool init();
    
    /**
     * @brief 启动 ROS2 spin
     */
    void start();
    
    /**
     * @brief 停止 ROS2
     */
    void stop();
    
    /**
     * @brief 设置 WebSocket 服务器（用于广播状态）
     */
    void set_server(Server* server) { server_ = server; }
    
    /**
     * @brief 发布 VR 控制意图
     * @param data 序列化的 VRControlIntent
     */
    void publish_vr_intent(const std::vector<uint8_t>& data);
    
    /**
     * @brief 发布底盘速度命令
     * @param linear_x 线速度 X
     * @param linear_y 线速度 Y
     * @param angular_z 角速度 Z
     */
    void publish_cmd_vel(double linear_x, double linear_y, double angular_z);
    
    /**
     * @brief 发布 Watchdog 心跳
     */
    void publish_watchdog_heartbeat();
    
    /**
     * @brief 获取 ROS2 节点
     */
    rclcpp::Node::SharedPtr get_node() { return node_; }
    
private:
    /**
     * @brief 关节状态回调
     */
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    /**
     * @brief 将关节状态转换为 Protobuf
     */
    std::vector<uint8_t> joint_state_to_proto(
        const sensor_msgs::msg::JointState::SharedPtr msg);
    
private:
    const Config& config_;
    StateCache& state_cache_;
    Server* server_ = nullptr;
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr watchdog_heartbeat_pub_;
    
    std::atomic<bool> running_{false};
    std::thread spin_thread_;
};

} // namespace qyh::dataplane

#endif // WITH_ROS2
