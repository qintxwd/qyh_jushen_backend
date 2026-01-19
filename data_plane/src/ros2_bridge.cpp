/**
 * @file ros2_bridge.cpp
 * @brief ROS2 桥接实现
 */

#ifdef WITH_ROS2

#include "data_plane/ros2_bridge.hpp"
#include "data_plane/config.hpp"
#include "data_plane/state_cache.hpp"
#include "data_plane/server.hpp"

#include "state.pb.h"
#include "messages.pb.h"

#include <iostream>

namespace qyh::dataplane {

ROS2Bridge::ROS2Bridge(const Config& config, StateCache& state_cache)
    : config_(config)
    , state_cache_(state_cache)
{
}

ROS2Bridge::~ROS2Bridge() {
    stop();
}

bool ROS2Bridge::init() {
    try {
        // 创建 ROS2 节点
        rclcpp::NodeOptions options;
        node_ = std::make_shared<rclcpp::Node>(config_.ros2.node_name, options);
        
        // 创建 executor
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        
        // QoS 配置
        auto state_qos = rclcpp::QoS(10)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        
        auto control_qos = rclcpp::QoS(10)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        
        // 订阅关节状态
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            state_qos,
            std::bind(&ROS2Bridge::joint_state_callback, this, std::placeholders::_1)
        );
        
        // 发布底盘速度
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            control_qos
        );
        
        // 发布 Watchdog 心跳
        watchdog_heartbeat_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/watchdog/heartbeat",
            control_qos
        );
        
        RCLCPP_INFO(node_->get_logger(), "ROS2 Bridge initialized");
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to init ROS2 Bridge: " << e.what() << std::endl;
        return false;
    }
}

void ROS2Bridge::start() {
    if (running_) {
        return;
    }
    
    running_ = true;
    
    // 启动 spin 线程
    spin_thread_ = std::thread([this]() {
        while (running_ && rclcpp::ok()) {
            executor_->spin_some(std::chrono::milliseconds(10));
        }
    });
    
    RCLCPP_INFO(node_->get_logger(), "ROS2 Bridge started");
}

void ROS2Bridge::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "ROS2 Bridge stopped");
    }
}

void ROS2Bridge::publish_vr_intent(const std::vector<uint8_t>& data) {
    // TODO: 解析 VRControlIntent 并发布到 ROS2
    // 需要定义对应的 ROS2 消息类型
    (void)data;
}

void ROS2Bridge::publish_cmd_vel(double linear_x, double linear_y, double angular_z) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.angular.z = angular_z;
    
    cmd_vel_pub_->publish(msg);
}

void ROS2Bridge::publish_watchdog_heartbeat() {
    std_msgs::msg::Bool msg;
    msg.data = true;
    watchdog_heartbeat_pub_->publish(msg);
}

void ROS2Bridge::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 转换为 Protobuf 并缓存
    auto proto_data = joint_state_to_proto(msg);
    state_cache_.update_joint_state(proto_data);
    
    // 广播到 WebSocket 订阅者
    if (server_) {
        // 包装为 WebSocketMessage
        qyh::proto::WebSocketMessage ws_msg;
        ws_msg.set_type(qyh::proto::MessageType::JOINT_STATE);
        
        // 设置时间戳
        auto* header = ws_msg.mutable_header();
        header->mutable_timestamp()->set_sec(
            static_cast<int32_t>(msg->header.stamp.sec));
        header->mutable_timestamp()->set_nsec(
            static_cast<int32_t>(msg->header.stamp.nanosec));
        
        // 序列化
        std::vector<uint8_t> data(ws_msg.ByteSizeLong());
        ws_msg.SerializeToArray(data.data(), static_cast<int>(data.size()));
        
        // 广播给订阅了 joint_state 话题的客户端
        server_->broadcast_to_subscribers("joint_state", data);
    }
}

std::vector<uint8_t> ROS2Bridge::joint_state_to_proto(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    
    qyh::proto::JointState proto;
    
    // 时间戳
    proto.mutable_timestamp()->set_sec(
        static_cast<int32_t>(msg->header.stamp.sec));
    proto.mutable_timestamp()->set_nsec(
        static_cast<int32_t>(msg->header.stamp.nanosec));
    
    // 关节名称
    for (const auto& name : msg->name) {
        proto.add_names(name);
    }
    
    // 位置
    for (const auto& pos : msg->position) {
        proto.add_positions(pos);
    }
    
    // 速度
    for (const auto& vel : msg->velocity) {
        proto.add_velocities(vel);
    }
    
    // 力矩
    for (const auto& eff : msg->effort) {
        proto.add_efforts(eff);
    }
    
    // 序列化
    std::vector<uint8_t> data(proto.ByteSizeLong());
    proto.SerializeToArray(data.data(), static_cast<int>(data.size()));
    
    return data;
}

} // namespace qyh::dataplane

#endif // WITH_ROS2
