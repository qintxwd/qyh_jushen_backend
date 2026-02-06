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

control_sync:
  # Control Plane 基础地址（与 Data Plane 同机时使用 127.0.0.1）
  control_plane_url: "http://127.0.0.1:8000"
  # Control Plane 内部接口鉴权 Token（需与 Control Plane 的 VR_INTERNAL_TOKEN 一致）
  internal_token: ""
  # 同步间隔（毫秒）
  sync_interval_ms: 1000
  # 请求超时（毫秒）
  timeout_ms: 5000
  # 是否启用同步
  enabled: true
```

### 内部接口鉴权

Data Plane 在上报 VR 连接状态时会调用 Control Plane 的内部接口。
若 Control Plane 配置了 `VR_INTERNAL_TOKEN`，必须在 Data Plane 配置相同的 Token。

支持的配置方式：
- 配置文件: `control_sync.internal_token`
- 环境变量: `CONTROL_PLANE_INTERNAL_TOKEN`

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

#### 认证与订阅
| 类型 | 值 | 方向 | 说明 |
|------|-----|------|------|
| `MSG_AUTH_REQUEST` | 1 | C→S | 认证请求 (携带 JWT) |
| `MSG_AUTH_RESPONSE` | 2 | S→C | 认证响应 |
| `MSG_SUBSCRIBE` | 16 | C→S | 订阅话题 |
| `MSG_UNSUBSCRIBE` | 17 | C→S | 取消订阅 |
| `MSG_HEARTBEAT` | 32 | C→S | 心跳 (< 200ms 间隔) |
| `MSG_HEARTBEAT_ACK` | 33 | S→C | 心跳响应 |

#### 控制意图 (客户端→服务端)
| 类型 | 值 | 说明 | Protobuf 消息 |
|------|-----|------|---------------|
| `MSG_VR_CONTROL` | 256 | VR 控制意图 | `VRControlIntent` |
| `MSG_CHASSIS_VELOCITY` | 257 | 底盘速度命令 | `ChassisVelocity` |
| `MSG_JOINT_COMMAND` | 258 | 关节位置命令 | `JointCommand` |
| `MSG_END_EFFECTOR_CMD` | 259 | 末端执行器命令 | `EndEffectorCommand` |
| `MSG_GRIPPER_COMMAND` | 260 | 夹爪命令 | `GripperCommand` |
| `MSG_NAVIGATION_GOAL` | 261 | 导航目标 | `NavigationGoal` |
| `MSG_NAVIGATION_CANCEL` | 262 | 取消导航 | `NavigationControl` |
| `MSG_NAVIGATION_PAUSE` | 263 | 暂停导航 | `NavigationControl` |
| `MSG_NAVIGATION_RESUME` | 264 | 恢复导航 | `NavigationControl` |
| `MSG_LIFT_COMMAND` | 265 | 升降控制 | `LiftCommand` |
| `MSG_WAIST_COMMAND` | 266 | 腰部控制 | `WaistCommand` |
| `MSG_HEAD_COMMAND` | 267 | 头部控制 | `HeadCommand` |
| `MSG_ARM_MOVE` | 268 | 机械臂运动 | `ArmMoveCommand` |
| `MSG_ARM_JOG` | 269 | 机械臂点动 | `ArmJogCommand` |
| `MSG_EMERGENCY_STOP` | 1026 | 紧急停止 | - |

#### 状态推送 (服务端→客户端)
| 类型 | 值 | 说明 | 推送频率 |
|------|-----|------|----------|
| `MSG_ROBOT_STATE` | 512 | 机器人综合状态 | 30Hz |
| `MSG_JOINT_STATE` | 513 | 关节状态 | 100Hz |
| `MSG_ARM_STATE` | 514 | 机械臂状态 | 50Hz |
| `MSG_CHASSIS_STATE` | 515 | 底盘状态 | 30Hz |
| `MSG_GRIPPER_STATE` | 516 | 夹爪状态 | 事件触发 |
| `MSG_VR_SYSTEM_STATE` | 517 | VR 系统状态 | 90Hz |
| `MSG_TASK_STATE` | 518 | 任务状态 | 事件触发 |
| `MSG_ACTUATOR_STATE` | 519 | 执行器状态 | 30Hz |

#### 系统通知
| 类型 | 值 | 方向 | 说明 |
|------|-----|------|------|
| `MSG_ERROR` | 768 | S→C | 错误通知 |
| `MSG_MODE_CHANGED` | 1024 | S→C | 模式变更通知 |
| `MSG_CONTROL_CHANGED` | 1025 | S→C | 控制权变更通知 |
| `MSG_EMERGENCY_STOP` | 1026 | S→C | 紧急停止通知 |

## 安全机制

### Watchdog (看门狗)

根据 `重构.md` 核心要求：

> ❌ 控制指令不设超时保护

**实现规则**:

1. 客户端必须每 **<200ms** 发送 `HEARTBEAT` 消息
2. 每个 WebSocket 会话独立跟踪心跳时间戳
3. 单个会话超时会被移除，所有会话都超时才触发紧急停止
4. 紧急停止动作：
   - 发布零速度到 `/cmd_vel` 话题
   - 调用用户自定义回调（如断开所有客户端连接）
   - 设置 `emergency_triggered_` 标志防止重复触发
5. 收到新心跳后重置紧急停止状态
6. 记录安全事件日志

**代码实现**:
```cpp
// include/data_plane/watchdog.hpp
class Watchdog {
    void feed(const std::string& session_id);  // 喂狗
    void unregister(const std::string& session_id);  // 注销会话
    void set_emergency_stop_callback(EmergencyStopCallback callback);
    void trigger_emergency_stop();  // 手动触发急停
};
```

**配置**:
```yaml
watchdog:
  timeout_ms: 200              # ⚠️ 超时触发紧急停止
  check_interval_ms: 50        # 检查间隔
```

### 紧急停止 (Emergency Stop)

**双通道冗余设计** (符合 ISO 10218 安全要求):

| 通道 | 触发方式 | 延迟 | 场景 |
|------|----------|------|------|
| **WebSocket (主)** | `MSG_EMERGENCY_STOP` 消息 | <10ms | 正常操作时的急停 |
| **HTTP (备)** | `POST /api/v1/emergency/stop` | 50-200ms | WebSocket 断开时的后备 |
| **Watchdog** | 心跳超时 200ms | 自动 | 连接异常时自动触发 |
| **ROS2 话题** | `/emergency_stop` | 取决于网络 | 与其他 ROS2 节点联动 |

**紧急停止流程**:
```
┌─────────────────┐
│   触发源        │
│ - WebSocket消息 │
│ - HTTP API      │
│ - Watchdog超时  │
│ - ROS2话题      │
└────────┬────────┘
         ▼
┌─────────────────┐
│  MessageHandler │
│  handle_emergency_stop()
└────────┬────────┘
         │
    ┌────┴────┬────────────┐
    ▼         ▼            ▼
┌───────┐ ┌────────┐ ┌──────────┐
│发布零 │ │触发    │ │广播急停  │
│速度   │ │Watchdog│ │通知给所  │
│/cmd_vel│ │回调    │ │有客户端  │
└───────┘ └────────┘ └──────────┘
```

**Protobuf 消息**:
```protobuf
// 紧急停止通知（双向）
message EmergencyStopNotification {
    Header header = 1;
    bool active = 2;           // true=急停激活, false=急停解除
    string source = 3;         // 触发来源: "websocket", "http", "watchdog", "hardware"
    string reason = 4;
}
```

**前端实现示例**:
```typescript
// WebSocket 急停 (推荐)
function emergencyStop() {
  const msg = WebSocketMessage.create({
    type: MessageType.MSG_EMERGENCY_STOP,
    timestamp: Date.now()
  });
  ws.send(WebSocketMessage.encode(msg).finish());
}

// HTTP 急停 (后备)
async function emergencyStopFallback() {
  await fetch('/api/v1/emergency/stop', {
    method: 'POST',
    headers: { 'Authorization': `Bearer ${token}` }
  });
}
```

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

## Protobuf 消息结构

### 执行器控制消息

```protobuf
// 升降控制命令
message LiftCommand {
    Header header = 1;
    string command = 2;         // "goto", "up", "down", "stop"
    double target_height = 3;   // 目标高度 (米), 仅 "goto" 时有效
    double speed = 4;           // 速度 (米/秒)
}

// 腰部控制命令
message WaistCommand {
    Header header = 1;
    string command = 2;         // "goto", "left", "right", "stop"
    double target_angle = 3;    // 目标角度 (弧度), 仅 "goto" 时有效
    double speed = 4;           // 角速度 (弧度/秒)
}

// 头部控制命令
message HeadCommand {
    Header header = 1;
    string command = 2;         // "goto", "preset", "track"
    double yaw = 3;             // 偏航角 (弧度)
    double pitch = 4;           // 俯仰角 (弧度)
    string preset_name = 5;     // 预设点名称
    double speed = 6;           // 速度因子 (0.0-1.0)
}
```

### 机械臂控制消息

```protobuf
// 机械臂运动命令 (MoveJ/MoveL)
message ArmMoveCommand {
    Header header = 1;
    string arm_side = 2;        // "left" 或 "right"
    string motion_type = 3;     // "movej" (关节空间) 或 "movel" (笛卡尔空间)
    repeated double target = 4; // 目标：关节角度或末端位姿 [x,y,z,rx,ry,rz]
    double speed = 5;           // 速度 (rad/s 或 m/s)
    double acceleration = 6;    // 加速度
    double blend_radius = 7;    // 平滑半径
}

// 机械臂点动命令 (Jog)
message ArmJogCommand {
    Header header = 1;
    string arm_side = 2;        // "left" 或 "right"
    string jog_mode = 3;        // "joint" (关节空间) 或 "cartesian" (笛卡尔空间)
    int32 axis_index = 4;       // 轴索引 (0-5)
    double direction = 5;       // 方向和速度 (-1.0 到 1.0)
}
```

### 导航控制消息

```protobuf
// 导航目标
message NavigationGoal {
    Header header = 1;
    Pose target_pose = 2;       // 目标位姿
    bool is_localization_only = 3; // 仅定位，不导航
}

// 导航控制 (取消/暂停/恢复)
message NavigationControl {
    Header header = 1;
    string action = 2;          // "cancel", "pause", "resume"
}
```

### 前端使用示例

```typescript
// TypeScript 示例 - 发送升降控制命令
import { WebSocketMessage, MessageType, LiftCommand } from './proto/messages';

function sendLiftCommand(command: string, height?: number, speed?: number) {
  const liftCmd = LiftCommand.create({
    command: command,
    targetHeight: height ?? 0,
    speed: speed ?? 0.1
  });
  
  const msg = WebSocketMessage.create({
    type: MessageType.MSG_LIFT_COMMAND,
    liftCommand: liftCmd
  });
  
  ws.send(WebSocketMessage.encode(msg).finish());
}

// 使用示例
sendLiftCommand('goto', 0.5, 0.1);  // 移动到 0.5m 高度
sendLiftCommand('up', undefined, 0.05);  // 以 0.05m/s 向上移动
sendLiftCommand('stop');  // 停止

// 机械臂 MoveL 示例
function sendArmMoveL(armSide: string, targetPose: number[]) {
  const armCmd = ArmMoveCommand.create({
    armSide: armSide,
    motionType: 'movel',
    target: targetPose,  // [x, y, z, rx, ry, rz]
    speed: 0.1,
    acceleration: 0.5
  });
  
  const msg = WebSocketMessage.create({
    type: MessageType.MSG_ARM_MOVE,
    armMove: armCmd
  });
  
  ws.send(WebSocketMessage.encode(msg).finish());
}
```

## 相关文档

- [重构.md](../重构.md) - 架构设计原则
- [重构开发计划.md](../重构开发计划.md) - 开发计划
- [Control Plane README](../control_plane/README.md)
- [Protobuf 定义](../shared/proto/)

---

*QYH Robot Data Plane v2.0.0*
