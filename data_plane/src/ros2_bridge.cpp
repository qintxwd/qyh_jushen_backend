/**
 * @file ros2_bridge.cpp
 * @brief ROS2 桥接实现
 * 
 * 完整实现机器人状态订阅和控制命令发布
 */

#include "data_plane/ros2_bridge.hpp"
#include "data_plane/config.hpp"
#include "data_plane/state_cache.hpp"
#include "data_plane/server.hpp"

#include "state.pb.h"
#include "control.pb.h"
#include "messages.pb.h"
#include "common.pb.h"

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
        
        // QoS 配置 - 状态数据使用 BestEffort
        auto state_qos = rclcpp::QoS(10)
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        
        // QoS 配置 - 控制命令使用 Reliable
        auto control_qos = rclcpp::QoS(10)
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile);
        
        // ==================== 创建订阅者 ====================
        
        // 关节状态
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", state_qos,
            std::bind(&ROS2Bridge::joint_state_callback, this, std::placeholders::_1)
        );
        
        // 里程计
        //         odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        //             "/odom", state_qos,
        //             std::bind(&ROS2Bridge::odom_callback, this, std::placeholders::_1)
        //         );
        
        // 左臂状态
        left_arm_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/left_arm/joint_states", state_qos,
            std::bind(&ROS2Bridge::left_arm_state_callback, this, std::placeholders::_1)
        );
        
        // 右臂状态
        right_arm_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/right_arm/joint_states", state_qos,
            std::bind(&ROS2Bridge::right_arm_state_callback, this, std::placeholders::_1)
        );
        
        // 升降状态
        lift_sub_ = node_->create_subscription<qyh_lift_msgs::msg::LiftState>(
            "/lift/state", state_qos,
            std::bind(&ROS2Bridge::lift_state_callback, this, std::placeholders::_1)
        );
        
        // 腰部状态
        waist_sub_ = node_->create_subscription<qyh_waist_msgs::msg::WaistState>(
            "/waist/state", state_qos,
            std::bind(&ROS2Bridge::waist_state_callback, this, std::placeholders::_1)
        );
        
        // 头部关节状态
        head_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/head/joint_states", state_qos,
            std::bind(&ROS2Bridge::head_state_callback, this, std::placeholders::_1)
        );
        
        // 左夹爪状态
        left_gripper_sub_ = node_->create_subscription<qyh_gripper_msgs::msg::GripperState>(
            "/left/gripper_state", state_qos,
            std::bind(&ROS2Bridge::left_gripper_state_callback, this, std::placeholders::_1)
        );
        
        // 右夹爪状态
        right_gripper_sub_ = node_->create_subscription<qyh_gripper_msgs::msg::GripperState>(
            "/right/gripper_state", state_qos,
            std::bind(&ROS2Bridge::right_gripper_state_callback, this, std::placeholders::_1)
        );
        
        // 机器人综合状态 (包含电池、急停等)
        robot_status_sub_ = node_->create_subscription<qyh_standard_robot_msgs::msg::StandardRobotStatus>(
            "/standard_robot_status", state_qos,
            std::bind(&ROS2Bridge::standard_robot_status_callback, this, std::placeholders::_1)
        );

        // JAKA Robot State (Arm High-level Status)
        jaka_state_sub_ = node_->create_subscription<qyh_jaka_control_msgs::msg::RobotState>(
            "/jaka/robot_state", state_qos,
            std::bind(&ROS2Bridge::jaka_robot_state_callback, this, std::placeholders::_1)
        );
        
        // ==================== 创建发布者 ====================
        
        // 底盘速度
        cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", control_qos
        );
        
        // 左臂轨迹
        left_arm_cmd_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/left_arm/joint_trajectory", control_qos
        );
        
        // 右臂轨迹
        right_arm_cmd_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/right_arm/joint_trajectory", control_qos
        );
        
        // 左末端执行器目标
        left_ee_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/left_arm/ee_target", control_qos
        );
        
        // 右末端执行器目标
        right_ee_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/right_arm/ee_target", control_qos
        );
        
        // 左夹爪命令
        left_gripper_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/left_gripper/command", control_qos
        );
        
        // 右夹爪命令
        right_gripper_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/right_gripper/command", control_qos
        );
        
        // 升降命令
        lift_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/lift/command", control_qos
        );
        
        // 腰部命令
        waist_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/waist/command", control_qos
        );
        
        // 头部 Pan 命令
        head_pan_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/head/pan/command", control_qos
        );
        
        // 头部 Tilt 命令
        head_tilt_cmd_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/head/tilt/command", control_qos
        );
        
        // 导航目标
        nav_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", control_qos
        );
        
        // Watchdog 心跳
        watchdog_heartbeat_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/watchdog/heartbeat", control_qos
        );
        
        // VR 意图（自定义消息，使用 Float64MultiArray 临时替代）
        vr_intent_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/vr/control_intent", control_qos
        );
        
        RCLCPP_INFO(node_->get_logger(), "ROS2 Bridge initialized with full topic support");
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "[ROS2Bridge] Failed to init: " << e.what() << std::endl;
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

// ==================== 控制命令发布 ====================

void ROS2Bridge::publish_vr_intent(const VRControlIntent& intent) {
    // 将 VR 意图转换为 ROS2 消息发布
    // 这里使用 Float64MultiArray 作为临时方案
    // 实际项目中应定义专用的 ROS2 消息类型
    
    std_msgs::msg::Float64MultiArray msg;
    
    // 头部位姿 (7 个值: x,y,z,qx,qy,qz,qw)
    if (intent.has_head_pose()) {
        const auto& pose = intent.head_pose();
        if (pose.has_position()) {
            msg.data.push_back(pose.position().x());
            msg.data.push_back(pose.position().y());
            msg.data.push_back(pose.position().z());
        }
        if (pose.has_orientation()) {
            msg.data.push_back(pose.orientation().x());
            msg.data.push_back(pose.orientation().y());
            msg.data.push_back(pose.orientation().z());
            msg.data.push_back(pose.orientation().w());
        }
    }
    
    // 左手控制器数据
    if (intent.has_left_hand() && intent.left_hand().active()) {
        const auto& hand = intent.left_hand();
        if (hand.has_pose() && hand.pose().has_position()) {
            msg.data.push_back(hand.pose().position().x());
            msg.data.push_back(hand.pose().position().y());
            msg.data.push_back(hand.pose().position().z());
        }
        if (hand.has_pose() && hand.pose().has_orientation()) {
            msg.data.push_back(hand.pose().orientation().x());
            msg.data.push_back(hand.pose().orientation().y());
            msg.data.push_back(hand.pose().orientation().z());
            msg.data.push_back(hand.pose().orientation().w());
        }
        msg.data.push_back(hand.trigger());
        msg.data.push_back(hand.grip());
    }
    
    // 右手控制器数据
    if (intent.has_right_hand() && intent.right_hand().active()) {
        const auto& hand = intent.right_hand();
        if (hand.has_pose() && hand.pose().has_position()) {
            msg.data.push_back(hand.pose().position().x());
            msg.data.push_back(hand.pose().position().y());
            msg.data.push_back(hand.pose().position().z());
        }
        if (hand.has_pose() && hand.pose().has_orientation()) {
            msg.data.push_back(hand.pose().orientation().x());
            msg.data.push_back(hand.pose().orientation().y());
            msg.data.push_back(hand.pose().orientation().z());
            msg.data.push_back(hand.pose().orientation().w());
        }
        msg.data.push_back(hand.trigger());
        msg.data.push_back(hand.grip());
    }
    
    vr_intent_pub_->publish(msg);
}

void ROS2Bridge::publish_cmd_vel(double linear_x, double linear_y, double angular_z) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.angular.z = angular_z;
    
    cmd_vel_pub_->publish(msg);
}

void ROS2Bridge::publish_joint_command(const JointCommand& cmd) {
    // 根据关节名称判断是左臂还是右臂
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = node_->now();
    
    for (const auto& name : cmd.names()) {
        traj.joint_names.push_back(name);
    }
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto& pos : cmd.positions()) {
        point.positions.push_back(pos);
    }
    for (const auto& vel : cmd.velocities()) {
        point.velocities.push_back(vel);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(1.0);
    
    traj.points.push_back(point);
    
    // 简单判断：如果包含 "left" 则发到左臂
    bool is_left = false;
    for (const auto& name : cmd.names()) {
        if (name.find("left") != std::string::npos) {
            is_left = true;
            break;
        }
    }
    
    if (is_left) {
        left_arm_cmd_pub_->publish(traj);
    } else {
        right_arm_cmd_pub_->publish(traj);
    }
}

void ROS2Bridge::publish_end_effector_command(const EndEffectorCommand& cmd) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "base_link";
    
    if (cmd.has_target_pose()) {
        const auto& pose = cmd.target_pose();
        if (pose.has_position()) {
            msg.pose.position.x = pose.position().x();
            msg.pose.position.y = pose.position().y();
            msg.pose.position.z = pose.position().z();
        }
        if (pose.has_orientation()) {
            msg.pose.orientation.x = pose.orientation().x();
            msg.pose.orientation.y = pose.orientation().y();
            msg.pose.orientation.z = pose.orientation().z();
            msg.pose.orientation.w = pose.orientation().w();
        }
    }
    
    if (cmd.arm_side() == "left") {
        left_ee_cmd_pub_->publish(msg);
    } else {
        right_ee_cmd_pub_->publish(msg);
    }
}

void ROS2Bridge::publish_gripper_command(const GripperCommand& cmd) {
    std_msgs::msg::Float64 msg;
    msg.data = cmd.position();
    
    if (cmd.gripper_id() == "left") {
        left_gripper_cmd_pub_->publish(msg);
    } else {
        right_gripper_cmd_pub_->publish(msg);
    }
}

void ROS2Bridge::publish_navigation_goal(const NavigationGoal& goal) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = "map";
    
    if (goal.has_target_pose()) {
        const auto& pose = goal.target_pose();
        if (pose.has_position()) {
            msg.pose.position.x = pose.position().x();
            msg.pose.position.y = pose.position().y();
            msg.pose.position.z = pose.position().z();
        }
        if (pose.has_orientation()) {
            msg.pose.orientation.x = pose.orientation().x();
            msg.pose.orientation.y = pose.orientation().y();
            msg.pose.orientation.z = pose.orientation().z();
            msg.pose.orientation.w = pose.orientation().w();
        }
    }
    
    nav_goal_pub_->publish(msg);
}

void ROS2Bridge::cancel_navigation() {
    // 发送零速度停止底盘
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.linear.y = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_msg);
    
    // TODO: 如果有 Nav2 action，可以取消 goal
    std::cout << "[ROS2Bridge] Navigation cancelled" << std::endl;
}

void ROS2Bridge::pause_navigation() {
    // 发送零速度暂停底盘
    geometry_msgs::msg::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.linear.y = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_msg);
    
    // TODO: 可以通过 Nav2 的暂停接口实现
    std::cout << "[ROS2Bridge] Navigation paused" << std::endl;
}

void ROS2Bridge::resume_navigation() {
    // TODO: 恢复导航需要记住之前的目标
    std::cout << "[ROS2Bridge] Navigation resumed" << std::endl;
}

void ROS2Bridge::publish_lift_command(const LiftCommand& cmd) {
    // 根据命令类型选择发布方式
    const std::string& command = cmd.command();
    
    if (command == "stop") {
        // 发布零速度或停止命令
        std_msgs::msg::Float64 msg;
        msg.data = 0.0;
        lift_cmd_pub_->publish(msg);
    } else if (command == "goto") {
        // 发送目标高度
        std_msgs::msg::Float64 msg;
        msg.data = cmd.target_height();
        lift_cmd_pub_->publish(msg);
    } else if (command == "up") {
        // 以指定速度向上移动
        std_msgs::msg::Float64 msg;
        msg.data = std::abs(cmd.speed());  // 正速度 = 向上
        lift_cmd_pub_->publish(msg);
    } else if (command == "down") {
        // 以指定速度向下移动
        std_msgs::msg::Float64 msg;
        msg.data = -std::abs(cmd.speed());  // 负速度 = 向下
        lift_cmd_pub_->publish(msg);
    }
}

void ROS2Bridge::publish_waist_command(const WaistCommand& cmd) {
    const std::string& command = cmd.command();
    
    if (command == "stop") {
        std_msgs::msg::Float64 msg;
        msg.data = 0.0;
        waist_cmd_pub_->publish(msg);
    } else if (command == "goto") {
        std_msgs::msg::Float64 msg;
        msg.data = cmd.target_angle();
        waist_cmd_pub_->publish(msg);
    } else if (command == "left") {
        std_msgs::msg::Float64 msg;
        msg.data = std::abs(cmd.speed());  // 正速度 = 向左
        waist_cmd_pub_->publish(msg);
    } else if (command == "right") {
        std_msgs::msg::Float64 msg;
        msg.data = -std::abs(cmd.speed());  // 负速度 = 向右
        waist_cmd_pub_->publish(msg);
    }
}

void ROS2Bridge::publish_head_command(const HeadCommand& cmd) {
    const std::string& command = cmd.command();
    
    if (command == "goto") {
        // 发送偏航角
        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = cmd.yaw();
        head_pan_cmd_pub_->publish(yaw_msg);
        
        // 发送俯仰角
        std_msgs::msg::Float64 pitch_msg;
        pitch_msg.data = cmd.pitch();
        head_tilt_cmd_pub_->publish(pitch_msg);
    } else if (command == "preset") {
        // TODO: 从预设点配置中查找位置
        std::cout << "[ROS2Bridge] Head preset: " << cmd.preset_name() << std::endl;
    } else if (command == "track") {
        // TODO: 人脸跟踪模式
        std::cout << "[ROS2Bridge] Head tracking mode" << std::endl;
    }
}

void ROS2Bridge::publish_arm_move_command(const ArmMoveCommand& cmd) {
    // 构建 JointTrajectory 消息
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = node_->now();
    
    // 根据手臂选择发布者
    bool is_left = (cmd.arm_side() == "left");
    
    // 设置关节名称 (假设 6 轴机械臂)
    std::string prefix = is_left ? "l_" : "r_";
    for (int i = 1; i <= 6; ++i) {
        traj.joint_names.push_back(prefix + "joint" + std::to_string(i));
    }
    
    // 添加轨迹点
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (double pos : cmd.target()) {
        point.positions.push_back(pos);
    }
    
    // 根据速度估算到达时间
    double estimated_time = 2.0;  // 默认 2 秒
    if (cmd.speed() > 0) {
        estimated_time = 1.0 / cmd.speed();  // 简单估算
    }
    point.time_from_start = rclcpp::Duration::from_seconds(estimated_time);
    
    traj.points.push_back(point);
    
    // 发布
    if (is_left) {
        left_arm_cmd_pub_->publish(traj);
    } else {
        right_arm_cmd_pub_->publish(traj);
    }
}

void ROS2Bridge::publish_arm_jog_command(const ArmJogCommand& cmd) {
    // Jog 模式：持续发送小增量位置或速度
    // 这里简化实现，实际应该发送到专用的 Jog 话题
    
    std::cout << "[ROS2Bridge] Arm jog: side=" << cmd.arm_side() 
              << ", mode=" << cmd.jog_mode()
              << ", axis=" << cmd.axis_index()
              << ", direction=" << cmd.direction() << std::endl;
    
    // TODO: 实现真正的 Jog 逻辑
    // 可以发送到 JAKA SDK 的 Jog 接口或 MoveIt 的 servo 接口
}

void ROS2Bridge::publish_watchdog_heartbeat() {
    std_msgs::msg::Bool msg;
    msg.data = true;
    watchdog_heartbeat_pub_->publish(msg);
}

// ==================== 状态回调函数 ====================

void ROS2Bridge::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    auto proto_data = joint_state_to_proto(msg);
    state_cache_.update_joint_state(proto_data);
    broadcast_state("joint_state", qyh::dataplane::MSG_JOINT_STATE, proto_data);
}

void ROS2Bridge::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 转换为 ChassisState protobuf
    qyh::dataplane::ChassisState chassis;
    
    // 时间戳
    chassis.mutable_header()->mutable_stamp()->set_seconds(msg->header.stamp.sec);
    chassis.mutable_header()->mutable_stamp()->set_nanos(msg->header.stamp.nanosec);
    
    // 位姿
    auto* odom_pose = chassis.mutable_odom();
    odom_pose->mutable_position()->set_x(msg->pose.pose.position.x);
    odom_pose->mutable_position()->set_y(msg->pose.pose.position.y);
    odom_pose->mutable_position()->set_z(msg->pose.pose.position.z);
    odom_pose->mutable_orientation()->set_x(msg->pose.pose.orientation.x);
    odom_pose->mutable_orientation()->set_y(msg->pose.pose.orientation.y);
    odom_pose->mutable_orientation()->set_z(msg->pose.pose.orientation.z);
    odom_pose->mutable_orientation()->set_w(msg->pose.pose.orientation.w);
    
    // 速度
    auto* vel = chassis.mutable_velocity();
    vel->mutable_linear()->set_x(msg->twist.twist.linear.x);
    vel->mutable_linear()->set_y(msg->twist.twist.linear.y);
    vel->mutable_linear()->set_z(msg->twist.twist.linear.z);
    vel->mutable_angular()->set_x(msg->twist.twist.angular.x);
    vel->mutable_angular()->set_y(msg->twist.twist.angular.y);
    vel->mutable_angular()->set_z(msg->twist.twist.angular.z);
    
    // 附加缓存的状态
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        chassis.set_battery_level(battery_level_);
        chassis.set_emergency_stop(emergency_stop_active_);
    }
    
    // 序列化
    std::vector<uint8_t> data(chassis.ByteSizeLong());
    chassis.SerializeToArray(data.data(), static_cast<int>(data.size()));
    
    state_cache_.update_chassis_state(data);
    broadcast_state("chassis_state", qyh::dataplane::MSG_CHASSIS_STATE, data);
}

void ROS2Bridge::jaka_robot_state_callback(const qyh_jaka_control_msgs::msg::RobotState::SharedPtr msg) {
    qyh::dataplane::ArmState state;
    state.mutable_header()->mutable_stamp()->set_seconds(msg->header.stamp.sec);
    state.mutable_header()->mutable_stamp()->set_nanos(msg->header.stamp.nanosec);
    
    // Status Flags
    state.set_connected(msg->connected);
    state.set_powered_on(msg->powered_on);
    state.set_enabled(msg->enabled);
    state.set_in_estop(msg->in_estop);
    state.set_in_error(msg->in_error);
    state.set_servo_mode(msg->servo_mode_enabled);
    state.set_error_message(msg->error_message);
    
    // Left/Right In Position
    state.set_left_in_position(msg->left_in_position);
    state.set_right_in_position(msg->right_in_position);

    // Positions
    for (const auto& p : msg->left_joint_positions) state.add_left_positions(p);
    for (const auto& p : msg->right_joint_positions) state.add_right_positions(p);
    
    // Cartesian Poses
    auto* lp = state.mutable_left_end_effector();
    lp->mutable_position()->set_x(msg->left_cartesian_pose.position.x);
    lp->mutable_position()->set_y(msg->left_cartesian_pose.position.y);
    lp->mutable_position()->set_z(msg->left_cartesian_pose.position.z);
    lp->mutable_orientation()->set_x(msg->left_cartesian_pose.orientation.x);
    lp->mutable_orientation()->set_y(msg->left_cartesian_pose.orientation.y);
    lp->mutable_orientation()->set_z(msg->left_cartesian_pose.orientation.z);
    lp->mutable_orientation()->set_w(msg->left_cartesian_pose.orientation.w);

    auto* rp = state.mutable_right_end_effector();
    rp->mutable_position()->set_x(msg->right_cartesian_pose.position.x);
    rp->mutable_position()->set_y(msg->right_cartesian_pose.position.y);
    rp->mutable_position()->set_z(msg->right_cartesian_pose.position.z);
    rp->mutable_orientation()->set_x(msg->right_cartesian_pose.orientation.x);
    rp->mutable_orientation()->set_y(msg->right_cartesian_pose.orientation.y);
    rp->mutable_orientation()->set_z(msg->right_cartesian_pose.orientation.z);
    rp->mutable_orientation()->set_w(msg->right_cartesian_pose.orientation.w);

    // Update Cache
    std::vector<uint8_t> data(state.ByteSizeLong());
    state.SerializeToArray(data.data(), static_cast<int>(data.size()));
    state_cache_.update_arm_state(data);
    
    // Broadcast if needed, or rely on aggregation
    broadcast_state("arm_state", qyh::dataplane::MSG_ARM_STATE, data);
}

void ROS2Bridge::left_arm_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 存储左臂关节状态
    auto proto_data = joint_state_to_proto(msg);
    state_cache_.update_left_arm_state(proto_data);
    // 不单独广播，在聚合的 arm_state 中推送
}

void ROS2Bridge::right_arm_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    auto proto_data = joint_state_to_proto(msg);
    state_cache_.update_right_arm_state(proto_data);
}

void ROS2Bridge::lift_state_callback(const qyh_lift_msgs::msg::LiftState::SharedPtr msg) {
    qyh::dataplane::ActuatorState state;
    state.set_actuator_id("lift");
    state.set_position(msg->current_position);
    state.set_velocity(msg->current_speed);
    state.set_in_motion(!msg->position_reached);
    state.set_in_position(msg->position_reached);
    
    std::vector<uint8_t> data(state.ByteSizeLong());
    state.SerializeToArray(data.data(), static_cast<int>(data.size()));
    
    state_cache_.update_lift_state(data);
    broadcast_state("actuator_state", qyh::dataplane::MSG_ACTUATOR_STATE, data);
}

void ROS2Bridge::waist_state_callback(const qyh_waist_msgs::msg::WaistState::SharedPtr msg) {
    qyh::dataplane::ActuatorState state;
    state.set_actuator_id("waist");
    // 优先使用 angle
    state.set_position(msg->current_angle);
    state.set_velocity(msg->current_speed);
    state.set_in_motion(!msg->position_reached);
    state.set_in_position(msg->position_reached);
    
    std::vector<uint8_t> data(state.ByteSizeLong());
    state.SerializeToArray(data.data(), static_cast<int>(data.size()));
    
    state_cache_.update_waist_state(data);
    broadcast_state("actuator_state", qyh::dataplane::MSG_ACTUATOR_STATE, data);
}

void ROS2Bridge::head_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 从 JointState 提取 Pan/Tilt
    // 假设关节名为 head_pan_joint, head_tilt_joint 或者包含 pan/tilt
    
    double pan_pos = 0.0, tilt_pos = 0.0;
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::string name = msg->name[i];
        if (name.find("pan") != std::string::npos || name.find("yaw") != std::string::npos) {
            if (i < msg->position.size()) pan_pos = msg->position[i];
        }
        else if (name.find("tilt") != std::string::npos || name.find("pitch") != std::string::npos) {
            if (i < msg->position.size()) tilt_pos = msg->position[i];
        }
    }

    // 1. Pan State
    {
        qyh::dataplane::ActuatorState state;
        state.set_actuator_id("head_pan");
        state.set_position(pan_pos);
        
        std::vector<uint8_t> data(state.ByteSizeLong());
        state.SerializeToArray(data.data(), static_cast<int>(data.size()));
        
        state_cache_.update_head_pan_state(data);
        broadcast_state("actuator_state", qyh::dataplane::MSG_ACTUATOR_STATE, data);
    }

    // 2. Tilt State
    {
        qyh::dataplane::ActuatorState state;
        state.set_actuator_id("head_tilt");
        state.set_position(tilt_pos);
        
        std::vector<uint8_t> data(state.ByteSizeLong());
        state.SerializeToArray(data.data(), static_cast<int>(data.size()));
        
        state_cache_.update_head_tilt_state(data);
        broadcast_state("actuator_state", qyh::dataplane::MSG_ACTUATOR_STATE, data);
    }
}

void ROS2Bridge::left_gripper_state_callback(const qyh_gripper_msgs::msg::GripperState::SharedPtr msg) {
    qyh::dataplane::GripperState state;
    state.set_gripper_id("left");
    state.set_position(msg->current_position / 255.0); // 归一化
    state.set_force(msg->current_force);
    state.set_object_detected(msg->object_status == 2);
    state.set_in_motion(msg->is_moving);
    
    std::vector<uint8_t> data(state.ByteSizeLong());
    state.SerializeToArray(data.data(), static_cast<int>(data.size()));
    
    state_cache_.update_gripper_state("left", data);
    broadcast_state("gripper_state", qyh::dataplane::MSG_GRIPPER_STATE, data);
}

void ROS2Bridge::right_gripper_state_callback(const qyh_gripper_msgs::msg::GripperState::SharedPtr msg) {
    qyh::dataplane::GripperState state;
    state.set_gripper_id("right");
    state.set_position(msg->current_position / 255.0);
    state.set_force(msg->current_force);
    state.set_object_detected(msg->object_status == 2);
    state.set_in_motion(msg->is_moving);
    
    std::vector<uint8_t> data(state.ByteSizeLong());
    state.SerializeToArray(data.data(), static_cast<int>(data.size()));
    
    state_cache_.update_gripper_state("right", data);
    broadcast_state("gripper_state", qyh::dataplane::MSG_GRIPPER_STATE, data);
}

void ROS2Bridge::standard_robot_status_callback(const qyh_standard_robot_msgs::msg::StandardRobotStatus::SharedPtr msg) {
    // 1. 更新电池
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        battery_level_ = msg->battery_remaining_percentage;
    }

    // 2. 更新急停
    bool current_estop = msg->is_emergency_stopped;
    bool prev_estop = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        prev_estop = emergency_stop_active_;
        emergency_stop_active_ = current_estop;
    }
    
    // 如果急停状态改变，广播通知
    if (current_estop != prev_estop) {
        qyh::dataplane::WebSocketMessage ws_msg;
        ws_msg.set_type(qyh::dataplane::MSG_EMERGENCY_STOP);
        
        auto* notification = ws_msg.mutable_emergency_stop();
        notification->set_active(current_estop);
        notification->set_source("hardware");
        
        std::vector<uint8_t> data(ws_msg.ByteSizeLong());
        ws_msg.SerializeToArray(data.data(), static_cast<int>(data.size()));
        
        if (server_) {
            server_->broadcast(data);
        }
    }
    
    // 3. 可能的底盘状态更新 (Pose/Twist)
    // 如果我们想用这里的 Pose/Twist 覆盖 odom，可以在这里做
    // 目前 ros2_bridge.cpp 依赖 odom_callback。如果 standard_robot_status 更准，可以考虑切换。
    // 暂时保持 odom，它是标准的。
}

// ==================== 辅助函数 ====================

std::vector<uint8_t> ROS2Bridge::joint_state_to_proto(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    
    qyh::dataplane::JointState proto;
    
    // 时间戳
    proto.mutable_header()->mutable_stamp()->set_seconds(msg->header.stamp.sec);
    proto.mutable_header()->mutable_stamp()->set_nanos(msg->header.stamp.nanosec);
    
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

void ROS2Bridge::broadcast_state(const std::string& topic_name, 
                                  int message_type,
                                  const std::vector<uint8_t>& payload) {
    if (!server_) {
        return;
    }
    
    // 包装为 WebSocketMessage
    qyh::dataplane::WebSocketMessage ws_msg;
    ws_msg.set_type(static_cast<qyh::dataplane::MessageType>(message_type));
    
    // 设置时间戳
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000;
    
    ws_msg.mutable_timestamp()->set_seconds(seconds);
    ws_msg.mutable_timestamp()->set_nanos(static_cast<int32_t>(nanos));
    
    // 零拷贝序列化
    auto data = std::make_shared<std::vector<uint8_t>>(ws_msg.ByteSizeLong());
    ws_msg.SerializeToArray(data->data(), static_cast<int>(data->size()));
    
    // 广播给订阅了该话题的客户端
    server_->broadcast_to_subscribers(topic_name, data);
}

} // namespace qyh::dataplane
