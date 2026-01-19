/**
 * @file ros2_bridge.hpp
 * @brief ROS2 桥接模块
 * 
 * 负责与 ROS2 系统通信，订阅状态话题并发布控制命令
 * 支持双臂机器人的完整状态监控和控制
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// Custom Messages
#include <qyh_lift_msgs/msg/lift_state.hpp>
#include <qyh_waist_msgs/msg/waist_state.hpp>
#include <qyh_gripper_msgs/msg/gripper_state.hpp>
#include <qyh_standard_robot_msgs/msg/standard_robot_status.hpp>
#include <qyh_jaka_control_msgs/msg/robot_state.hpp>

#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>

// 前向声明 Protobuf 类型
namespace qyh::dataplane {
    class VRControlIntent;
    class JointCommand;
    class EndEffectorCommand;
    class GripperCommand;
    class NavigationGoal;
}

namespace qyh::dataplane {

// 前向声明
class Config;
class StateCache;
class Server;

/**
 * @brief ROS2 桥接
 * 
 * 完整实现机器人状态订阅和控制命令发布
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
    
    // ==================== 控制命令发布 ====================
    
    /**
     * @brief 发布 VR 控制意图
     * @param intent VR 控制意图消息
     */
    void publish_vr_intent(const VRControlIntent& intent);
    
    /**
     * @brief 发布底盘速度命令
     */
    void publish_cmd_vel(double linear_x, double linear_y, double angular_z);
    
    /**
     * @brief 发布关节命令
     */
    void publish_joint_command(const JointCommand& cmd);
    
    /**
     * @brief 发布末端执行器命令
     */
    void publish_end_effector_command(const EndEffectorCommand& cmd);
    
    /**
     * @brief 发布夹爪命令
     */
    void publish_gripper_command(const GripperCommand& cmd);
    
    /**
     * @brief 发布导航目标
     */
    void publish_navigation_goal(const NavigationGoal& goal);
    
    /**
     * @brief 发布 Watchdog 心跳
     */
    void publish_watchdog_heartbeat();
    
    /**
     * @brief 获取 ROS2 节点
     */
    rclcpp::Node::SharedPtr get_node() { return node_; }
    
private:
    // ==================== 状态回调函数 ====================
    
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void left_arm_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void right_arm_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void lift_state_callback(const qyh_lift_msgs::msg::LiftState::SharedPtr msg);
    void waist_state_callback(const qyh_waist_msgs::msg::WaistState::SharedPtr msg);
    void head_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void left_gripper_state_callback(const qyh_gripper_msgs::msg::GripperState::SharedPtr msg);
    void right_gripper_state_callback(const qyh_gripper_msgs::msg::GripperState::SharedPtr msg);
    void standard_robot_status_callback(const qyh_standard_robot_msgs::msg::StandardRobotStatus::SharedPtr msg);
    void jaka_robot_state_callback(const qyh_jaka_control_msgs::msg::RobotState::SharedPtr msg);
    
    // ==================== 数据转换函数 ====================
    
    std::vector<uint8_t> joint_state_to_proto(
        const sensor_msgs::msg::JointState::SharedPtr msg);
    
    void broadcast_state(const std::string& topic_name, 
                         int message_type,
                         const std::vector<uint8_t>& data);
    
private:
    const Config& config_;
    StateCache& state_cache_;
    Server* server_ = nullptr;
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    
    // ==================== 订阅者 ====================
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_arm_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_arm_sub_;
    rclcpp::Subscription<qyh_lift_msgs::msg::LiftState>::SharedPtr lift_sub_;
    rclcpp::Subscription<qyh_waist_msgs::msg::WaistState>::SharedPtr waist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr head_sub_;
    rclcpp::Subscription<qyh_gripper_msgs::msg::GripperState>::SharedPtr left_gripper_sub_;
    rclcpp::Subscription<qyh_gripper_msgs::msg::GripperState>::SharedPtr right_gripper_sub_;
    rclcpp::Subscription<qyh_standard_robot_msgs::msg::StandardRobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<qyh_jaka_control_msgs::msg::RobotState>::SharedPtr jaka_state_sub_;
    
    // ==================== 发布者 ====================
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_arm_cmd_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_arm_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_ee_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_ee_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_gripper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_gripper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lift_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr waist_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr head_pan_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr head_tilt_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr watchdog_heartbeat_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vr_intent_pub_;
    
    std::atomic<bool> running_{false};
    std::thread spin_thread_;
    
    // 状态缓存（用于聚合推送）
    std::mutex state_mutex_;
    double battery_level_ = 0.0;
    bool emergency_stop_active_ = false;
};

} // namespace qyh::dataplane
