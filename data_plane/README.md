# QYH Robot Data Plane

> **数据平面** - 基于 C++ / Boost.Beast / ROS2 的高性能 WebSocket 服务器
>
> 负责高频状态推送和实时控制意图传输

## 架构定位

```
┌─────────────────────────────────────────────────────────────┐
│                      Client Layer                           │
│              Web UI / VR / 运维面板                          │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────────┐
│                       API Layer                             │
│                                                             │
│   Control Plane          Data Plane           Media Plane   │
│   (FastAPI:8000)      (WebSocket:8765)    (WebRTC:8888)    │
│   - 认证/配置             - 实时状态           - 视频流      │
│   - 任务管理              - 控制意图           - 多相机      │
│   - 模式切换              - Watchdog                        │
│                                                             │
└────────────────────────┬────────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────────┐
│                    ROS2 Runtime Layer                       │
│          Drivers / Control Loop / Safety                    │
└─────────────────────────────────────────────────────────────┘
```

## 核心职责

| 功能 | 说明 |
|------|------|
| **实时状态推送** | 关节状态、TF、IMU 等高频数据 (30-100Hz) |
| **控制意图接收** | VR 遥操、底盘速度、关节命令 |
| **Watchdog 监控** | 200ms 超时保护，自动触发紧急停止 |
| **JWT 认证** | 与 Control Plane 共享密钥 |
| **话题订阅** | 客户端按需订阅状态数据 |

## 技术栈

- **语言**: C++17
- **网络**: Boost.Beast (WebSocket), Boost.Asio
- **序列化**: Protobuf
- **中间件**: ROS2 (可选编译)
- **配置**: yaml-cpp

## 目录结构

```
data_plane/
├── CMakeLists.txt              # 构建配置
├── config/
│   └── config.yaml             # 运行时配置
├── include/data_plane/         # 头文件
│   ├── auth.hpp                # JWT 认证
│   ├── chassis_manual.hpp      # 底盘手动控制（摇杆/键盘）
│   ├── config.hpp              # 配置管理
│   ├── connection_manager.hpp  # 连接管理（限制/优先级）
│   ├── control_sync.hpp        # 控制权同步
│   ├── imu_tf_led.hpp          # IMU/TF/LED 支持
│   ├── message_handler.hpp     # 消息处理
│   ├── metrics.hpp             # Prometheus 监控指标
│   ├── performance.hpp         # 性能优化（内存池/缓存）
│   ├── rate_limiter.hpp        # 频率限流和差量更新
│   ├── ros2_bridge.hpp         # ROS2 桥接
│   ├── server.hpp              # WebSocket 服务器
│   ├── session.hpp             # 会话管理
│   ├── state_cache.hpp         # 状态缓存
│   └── watchdog.hpp            # 安全看门狗
└── src/                        # 源文件
    ├── main.cpp
    ├── auth.cpp
    ├── config.cpp
    ├── connection_manager.cpp
    ├── control_sync.cpp
    ├── message_handler.cpp
    ├── rate_limiter.cpp
    ├── ros2_bridge.cpp
    ├── server.cpp
    ├── session.cpp
    ├── state_cache.cpp
    └── watchdog.cpp
```

## 构建

### 依赖

- Boost >= 1.74
- Protobuf
- yaml-cpp
- ROS2 Humble (可选)

### 编译

```bash
# 不带 ROS2
mkdir build && cd build
cmake ..
make -j$(nproc)

# 带 ROS2 (在 ROS2 环境中)
source /opt/ros/humble/setup.bash
colcon build --packages-select qyh_data_plane
```

### 运行

```bash
# 使用默认配置
./qyh_data_plane

# 指定配置文件
./qyh_data_plane /etc/qyh-robot/data_plane/config.yaml
```

## 配置文件

```yaml
server:
  host: "0.0.0.0"
  port: 8765
  max_connections: 50

auth:
  jwt_secret: "${JWT_SECRET}"  # 从环境变量读取
  auth_timeout_sec: 10

watchdog:
  timeout_ms: 200              # ⚠️ 超时触发紧急停止
  check_interval_ms: 50

state_publish:
  robot_state_hz: 30
  joint_state_hz: 100
```

## WebSocket 协议

### 连接流程

```
Client                          Server
   |                               |
   |-------- Connect ------------->|
   |                               |
   |<------- Connected ------------|
   |                               |
   |--- AUTH_REQUEST (JWT) ------->|  // 必须在 10s 内认证
   |                               |
   |<-- AUTH_RESPONSE (success) ---|
   |                               |
   |--- SUBSCRIBE_REQUEST -------->|  // 订阅话题
   |                               |
   |<-- STATE (robot_state) -------|  // 开始推送
   |<-- STATE (joint_state) -------|
   |                               |
   |--- HEARTBEAT ---------------->|  // 每 <200ms 必须发送
   |                               |
```

### 消息类型 (Protobuf)

| 类型 | 方向 | 说明 |
|------|------|------|
| `AUTH_REQUEST` | C→S | 认证请求 (携带 JWT) |
| `AUTH_RESPONSE` | S→C | 认证响应 |
| `SUBSCRIBE_REQUEST` | C→S | 订阅话题 |
| `UNSUBSCRIBE_REQUEST` | C→S | 取消订阅 |
| `HEARTBEAT` | C→S | 心跳 (< 200ms 间隔) |
| `STATE` | S→C | 状态数据 (关节/TF/IMU) |
| `VR_CONTROL_INTENT` | C→S | VR 控制意图 |
| `CHASSIS_VELOCITY` | C→S | 底盘速度命令 |
| `JOINT_COMMAND` | C→S | 关节命令 |
| `ERROR` | S→C | 错误通知 |

## 安全机制

### Watchdog (看门狗)

根据 `重构.md` 核心要求：

> ❌ 控制指令不设超时保护

**实现规则**:

1. 客户端必须每 **<200ms** 发送 `HEARTBEAT`
2. 超时后自动发布 **紧急停止** 到 ROS2
3. 断开超时客户端连接
4. 记录安全事件日志

### JWT 认证

- 与 Control Plane 共享密钥
- 连接后 10 秒内必须完成认证
- 支持角色和权限检查

## 性能指标

| 指标 | 目标值 |
|------|--------|
| 状态推送延迟 | < 10ms |
| 控制意图延迟 | < 5ms |
| 最大并发连接 | 100 |
| CPU 占用 | < 10% (Jetson Orin) |
| 内存占用 | < 100MB |

## 高级功能

### 频率限流 (rate_limiter)

```cpp
// 配置话题最大频率
StateAggregator aggregator;
aggregator.configure_topic("joint_state", 100.0);  // 100Hz
aggregator.configure_topic("robot_state", 30.0);   // 30Hz
aggregator.set_delta_only("robot_state", true);    // 差量更新
```

### 连接管理 (connection_manager)

```cpp
// 优先级机制
ConnectionManager mgr(config);
auto result = mgr.try_accept("192.168.1.100", ConnectionPriority::HIGH);
if (!result.accepted) {
    // 处理拒绝
}
```

### 监控指标 (metrics)

```cpp
// Prometheus 格式导出
auto& metrics = MetricsCollector::instance();
metrics.connections_total.set(42);
metrics.messages_sent.inc();
std::string prometheus_output = metrics.export_prometheus();
```

### IMU/TF/LED (imu_tf_led)

```cpp
// LED 控制
LedController led;
led.warning();  // 红色闪烁
led.ready();    // 绿色常亮
led.running();  // 蓝色呼吸
```

### 底盘手动控制 (chassis_manual)

```cpp
// 创建控制器
ChassisManualController controller;
controller.set_velocity_callback([](const VelocityOutput& vel) {
    // 发布到 /cmd_vel
});

// 处理摇杆输入（游戏手柄/VR控制器）
JoystickInput joystick{0.5, 0.8, 0.0, 1.0};
auto vel = controller.process_joystick(joystick);

// 处理键盘输入
KeyboardInput keyboard;
keyboard.forward = true;
keyboard.speed_level = KeyboardInput::SpeedLevel::NORMAL;
auto vel2 = controller.process_keyboard(keyboard);
```

## 相关文档

- [重构.md](../重构.md) - 架构设计原则
- [重构开发计划.md](../重构开发计划.md) - 开发计划
- [Control Plane README](../control_plane/README.md)
- [Protobuf 定义](../shared/proto/)

---

*QYH Robot Data Plane v2.0.0*
