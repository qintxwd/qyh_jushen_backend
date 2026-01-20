# QYH Robot Backend 重构项目

## 项目结构

```
qyh_jushen_backend/
├── shared/                     # 共享定义
│   └── proto/                  # Protobuf 消息定义
│       ├── common.proto        # 通用类型
│       ├── control.proto       # 控制命令
│       ├── state.proto         # 状态数据
│       └── messages.proto      # WebSocket 消息封装
│
├── control_plane/              # 管理平面 (Python/FastAPI)
│   ├── app/
│   │   ├── main.py            # 应用入口
│   │   ├── config.py          # 配置管理
│   │   ├── database.py        # 数据库
│   │   ├── core/              # 核心逻辑
│   │   │   ├── security.py    # JWT 认证
│   │   │   ├── control_lock.py # 控制锁
│   │   │   └── mode_manager.py # 模式管理
│   │   ├── models/            # 数据模型
│   │   ├── schemas/           # Pydantic Schema
│   │   └── api/v1/            # API 路由
│   └── requirements.txt
│
├── data_plane/                 # 数据平面 (C++/Boost.Beast/ROS2)
│   ├── include/data_plane/    # 头文件
│   ├── src/                   # 源文件
│   ├── config/config.yaml     # 配置
│   └── CMakeLists.txt
│
├── media_plane/                # 媒体平面 (C++/GStreamer/WebRTC)
│   ├── include/media_plane/   # 头文件
│   ├── src/                   # 源文件
│   ├── config/config.yaml     # 配置
│   └── CMakeLists.txt
│
└── deploy/                     # 部署配置
    ├── systemd/               # Systemd 服务文件
    └── install.sh             # 安装脚本
```

## ⚠️ 接口职责严格分离（核心设计原则）

**本项目采用三平面架构，每种协议有严格的职责边界，禁止混用！**

### 协议分配规则

| 协议 | 频率 | 延迟要求 | 适用场景 |
|------|------|----------|----------|
| **HTTP (FastAPI)** | <5Hz | 100-500ms | 配置管理、CRUD、认证 |
| **WebSocket** | 30-100Hz | <20ms | 实时控制、状态推送、心跳 |
| **WebRTC** | 30fps | <100ms | 视频流 |

### ✅ 必须走 Control Plane (HTTP/FastAPI) 的接口

| 功能分类 | 接口 |
|----------|------|
| **认证** | 登录、登出、Token 刷新 |
| **控制权管理** | 获取、释放、续约控制权 |
| **模式切换** | 切换工作模式 (idle/teleop/auto) |
| **任务 CRUD** | 任务创建、编辑、删除、列表 |
| **预设 CRUD** | 预设创建、编辑、删除、应用 |
| **录制控制** | 启动/停止录制 |
| **配置管理** | 底盘配置、LED 颜色、系统配置 |
| **信息查询** | 机器人信息、站点列表、审计日志 |
| **系统管理** | 关机、重启 |

### ✅ 必须走 Data Plane (WebSocket) 的接口

| 功能分类 | 消息类型 | 频率 |
|----------|----------|------|
| **底盘速度** | `CHASSIS_VELOCITY` | 50Hz |
| **急停** | `EMERGENCY_STOP` | 事件触发 |
| **关节控制** | `JOINT_COMMAND` | 50-100Hz |
| **夹爪控制** | `GRIPPER_COMMAND` | 事件触发 |
| **导航命令** | `NAVIGATE_TO_POSE` | 事件触发 |
| **心跳** | `HEARTBEAT` | >5Hz (每200ms) |
| **状态推送** | `ROBOT_STATE`, `CHASSIS_STATE` | 30-100Hz |
| **VR 遥操作** | `VR_CONTROL_INTENT` | 50-90Hz |

### ✅ 必须走 Media Plane (WebRTC) 的接口

| 功能 |
|------|
| 视频流传输 |
| 音频流传输 (可选) |

### ❌ 禁止的设计

```
❌ 在 FastAPI 中提供速度命令接口      → 延迟太高
❌ 在 FastAPI 中提供急停接口          → 延迟不可接受
❌ 在 FastAPI 中提供关节控制接口      → 延迟太高
❌ 通过 HTTP 轮询获取实时状态         → 效率低、延迟高
❌ 通过 HTTP 传输视频帧              → 性能灾难
```

---

## 架构说明

### 三平面分离

| 平面 | 职责 | 技术栈 | 端口 |
|------|------|--------|------|
| Control Plane | 认证、配置、任务管理 | Python/FastAPI | 8000 |
| Data Plane | 实时控制、状态推送 | C++/Beast/ROS2 | 8765 |
| Media Plane | 视频流传输 | C++/GStreamer/WebRTC | 8888 |

### 服务发现

前端只需要知道 Control Plane 的地址，通过 `GET /api/v1/system/config` 获取其他服务地址：

```json
{
    "ws_data_url": "ws://robot:8765",
    "ws_media_url": "ws://robot:8888",
    "webrtc_enabled": true
}
```

### 认证流程

1. 前端通过 Control Plane 登录获取 JWT Token
2. 连接 Data Plane WebSocket 时发送 AuthRequest（包含 Token）
3. Data Plane 验证 Token 后允许订阅/发送消息
4. Media Plane 信令 WebSocket 同样需要认证

## 快速开始

### 开发环境

```bash
# Control Plane
cd control_plane
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt
uvicorn app.main:app --reload

# Data Plane (需要 ROS2 环境)
cd data_plane
mkdir build && cd build
cmake ..
make

# Media Plane
cd media_plane
mkdir build && cd build
cmake ..
make
```

### 生产部署

```bash
sudo ./deploy/install.sh
sudo systemctl start qyh-control-plane qyh-data-plane qyh-media-plane
```

## API 端点

### Control Plane (REST)

**认证与系统**
- `POST /api/v1/auth/login` - 登录
- `POST /api/v1/auth/logout` - 登出
- `POST /api/v1/auth/refresh` - 刷新 Token
- `GET /api/v1/system/config` - 服务发现
- `GET /api/v1/system/health` - 健康检查

**控制权与模式**
- `POST /api/v1/control/acquire` - 获取控制权
- `POST /api/v1/control/release` - 释放控制权
- `POST /api/v1/mode/switch` - 切换模式
- `GET /api/v1/mode/current` - 当前模式

**紧急停止 (HTTP 备用通道)**
- `POST /api/v1/emergency/stop` - 触发紧急停止
- `GET /api/v1/emergency/status` - 获取急停状态

**导航控制**
- `POST /api/v1/chassis/navigate/pose` - 导航到坐标点
- `POST /api/v1/chassis/navigate/station` - 导航到站点
- `POST /api/v1/chassis/navigate/cancel` - 取消导航
- `GET /api/v1/chassis/stations` - 获取站点列表
- `GET /api/v1/chassis/status` - 获取底盘状态

**摄像头**
- `GET /api/v1/camera/list` - 获取摄像头列表
- `GET /api/v1/camera/{id}` - 获取摄像头信息
- `GET /api/v1/camera/{id}/webrtc` - WebRTC 连接信息

**任务与预设**
- `GET /api/v1/tasks` - 任务列表
- `POST /api/v1/tasks` - 创建任务
- `GET /api/v1/presets` - 预设列表
- `POST /api/v1/presets/{type}/{id}/apply` - 应用预设

**录制与动作**
- `POST /api/v1/recording/start` - 开始录制
- `POST /api/v1/recording/stop` - 停止录制
- `GET /api/v1/actions` - 动作列表

### Data Plane (WebSocket + Protobuf)

连接: `ws://host:8765`

**认证与订阅 (0x0001-0x001F)**
| 消息类型 | 值 | 方向 | 说明 |
|----------|-----|------|------|
| `MSG_AUTH_REQUEST` | 1 | C→S | 认证请求 (携带 JWT) |
| `MSG_AUTH_RESPONSE` | 2 | S→C | 认证响应 |
| `MSG_SUBSCRIBE` | 16 | C→S | 订阅话题 |
| `MSG_UNSUBSCRIBE` | 17 | C→S | 取消订阅 |
| `MSG_HEARTBEAT` | 32 | C→S | 心跳 (<200ms 间隔) |
| `MSG_HEARTBEAT_ACK` | 33 | S→C | 心跳响应 |

**控制意图 (0x0100-0x01FF)**
| 消息类型 | 值 | 方向 | 说明 |
|----------|-----|------|------|
| `MSG_VR_CONTROL` | 256 | C→S | VR 控制意图 |
| `MSG_CHASSIS_VELOCITY` | 257 | C→S | 底盘速度命令 |
| `MSG_JOINT_COMMAND` | 258 | C→S | 关节位置命令 |
| `MSG_END_EFFECTOR_CMD` | 259 | C→S | 末端执行器命令 |
| `MSG_GRIPPER_COMMAND` | 260 | C→S | 夹爪命令 |
| `MSG_NAVIGATION_GOAL` | 261 | C→S | 导航目标 |
| `MSG_NAVIGATION_CANCEL` | 262 | C→S | 取消导航 |
| `MSG_NAVIGATION_PAUSE` | 263 | C→S | 暂停导航 |
| `MSG_NAVIGATION_RESUME` | 264 | C→S | 恢复导航 |
| `MSG_LIFT_COMMAND` | 265 | C→S | 升降控制 |
| `MSG_WAIST_COMMAND` | 266 | C→S | 腰部控制 |
| `MSG_HEAD_COMMAND` | 267 | C→S | 头部控制 |
| `MSG_ARM_MOVE` | 268 | C→S | 机械臂运动 (MoveJ/MoveL) |
| `MSG_ARM_JOG` | 269 | C→S | 机械臂点动 (Jog) |

**状态推送 (0x0200-0x02FF)**
| 消息类型 | 值 | 方向 | 说明 |
|----------|-----|------|------|
| `MSG_ROBOT_STATE` | 512 | S→C | 机器人综合状态 |
| `MSG_JOINT_STATE` | 513 | S→C | 关节状态 |
| `MSG_ARM_STATE` | 514 | S→C | 机械臂状态 |
| `MSG_CHASSIS_STATE` | 515 | S→C | 底盘状态 |
| `MSG_GRIPPER_STATE` | 516 | S→C | 夹爪状态 |
| `MSG_VR_SYSTEM_STATE` | 517 | S→C | VR 系统状态 |
| `MSG_TASK_STATE` | 518 | S→C | 任务状态 |
| `MSG_ACTUATOR_STATE` | 519 | S→C | 执行器状态 |

**系统通知 (0x0400-0x04FF)**
| 消息类型 | 值 | 方向 | 说明 |
|----------|-----|------|------|
| `MSG_ERROR` | 768 | S→C | 错误通知 |
| `MSG_MODE_CHANGED` | 1024 | S→C | 模式变更通知 |
| `MSG_CONTROL_CHANGED` | 1025 | S→C | 控制权变更通知 |
| `MSG_EMERGENCY_STOP` | 1026 | 双向 | 紧急停止 |

### Media Plane (WebSocket + JSON)

连接: `ws://host:8888`

信令消息:
- `request_stream` - 请求视频流
- `offer/answer` - SDP 交换
- `ice_candidate` - ICE 候选

## 重要约束

1. **Watchdog 超时**: 客户端必须每 200ms 发送心跳，否则触发紧急停止
2. **紧急停止双通道**: 
   - **主通道 (WebSocket)**: `MSG_EMERGENCY_STOP` 消息，延迟 <10ms
   - **备用通道 (HTTP)**: `POST /api/v1/emergency/stop`，用于 WebSocket 断开时
   - **自动触发**: Watchdog 超时自动触发
3. **控制锁**: 同一时间只有一个客户端可以控制机器人
4. **模式状态机**: 模式切换需遵循状态机规则
5. **视频不走 HTTP**: 视频必须通过 WebRTC 传输
6. **实时控制不走 HTTP**: 速度命令、关节控制必须走 WebSocket
7. **执行器控制**: 升降、腰部、头部、机械臂等执行器控制通过 WebSocket 实现

### 接口频率要求

| 接口类型 | 频率 | 延迟要求 | 协议 |
|----------|------|----------|------|
| 认证/配置 | <5Hz | 100-500ms | HTTP |
| 底盘速度 | 50Hz | <20ms | WebSocket |
| 机械臂控制 | 50-100Hz | <10ms | WebSocket |
| 心跳 | >5Hz | <200ms | WebSocket |
| 状态推送 | 30-100Hz | <10ms | WebSocket |
| 视频流 | 30fps | <100ms | WebRTC |

## 配置文件

- `/etc/qyh-robot/environment` - 共享环境变量
- `/etc/qyh-robot/data_plane/config.yaml` - Data Plane 配置
- `/etc/qyh-robot/media_plane/config.yaml` - Media Plane 配置
