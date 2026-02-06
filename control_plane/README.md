# QYH Jushen Control Plane

机器人后端服务的控制平面，使用 FastAPI 构建。负责处理低频管理请求（<5Hz），如认证、配置、任务、预设等。

## ⚠️ 重要：接口职责严格分离

本项目采用三平面架构，**每种接口类型有严格的职责边界，禁止混用**：

### 三平面架构

| 平面 | 语言 | 端口 | 频率 | 职责 |
|------|------|------|------|------|
| **Control Plane** | Python/FastAPI | 8000 | <5Hz | 低频管理 API（本模块） |
| **Data Plane** | C++/WebSocket | 8765 | 30-100Hz | 高频实时控制数据流 |
| **Media Plane** | C++/GStreamer+WebRTC | 8888 | 30fps | 视频流传输 |

### 🟢 Control Plane (HTTP/FastAPI) 接口职责

**适用场景：低频管理操作、需要事务性和审计的操作**

| 模块 | 接口 | 说明 |
|------|------|------|
| 认证 | `/auth/login`, `/auth/logout`, `/auth/refresh` | 登录认证 |
| 控制权 | `/control/acquire`, `/release`, `/renew` | 控制权管理（非实时控制本身） |
| 模式 | `/mode/switch`, `/mode/current` | 工作模式切换 |
| 任务 | `/tasks/*` CRUD | 任务创建、编辑、删除 |
| 预设 | `/presets/*` CRUD | 预设创建、编辑、删除 |
| 录制 | `/recording/start`, `/stop` | 录制控制（启停） |
| 底盘 | `/chassis/config`, `/chassis/stations` | 底盘配置读写、站点列表 |
| LED | `/led/color`, `/led/blink` | LED 颜色设置（低频操作） |
| 系统 | `/system/config`, `/robot/info`, `/robot/shutdown` | 系统配置、机器人信息、关机 |
| 审计 | `/audit/*` | 操作日志查询 |

### 🔴 禁止在 Control Plane 实现的接口

以下接口 **必须通过 Data Plane WebSocket 实现**，禁止在 FastAPI 中提供：

| 功能 | 原因 | WebSocket 消息类型 |
|------|------|------|
| 速度命令 | 需要 50Hz+ 实时性 | `CHASSIS_VELOCITY` |
| 急停触发 | 延迟必须最低 | `EMERGENCY_STOP` |
| 关节控制 | 需要 50Hz+ 实时性 | `JOINT_COMMAND` |
| 夹爪控制 | 需要低延迟 | `GRIPPER_COMMAND` |
| 心跳 | 需要持续连接检测 | `HEARTBEAT` |
| 实时状态推送 | 高频数据流 | `ROBOT_STATE`, `CHASSIS_STATE` |
| 导航命令 | 需要低延迟响应 | `NAVIGATE_TO_POSE` |

### 🔵 Media Plane (WebRTC) 接口职责

| 功能 | 说明 |
|------|------|
| 视频流 | 机器人摄像头视频推送 |
| 音频流 | 双向语音通话（可选） |

## 为什么严格分离？

```
❌ 错误设计：前端通过 HTTP 发送速度命令
   问题：HTTP 请求-响应延迟 50-200ms，无法满足实时控制

✅ 正确设计：前端通过 WebSocket 发送速度命令
   优点：持久连接，延迟 <10ms，支持双向通信
```

**规则**：
1. 凡是需要 >10Hz 频率的操作，必须走 WebSocket
2. 凡是需要 <20ms 延迟的操作（如急停），必须走 WebSocket
3. 只有低频的管理操作（CRUD、配置、认证）才走 HTTP

## 目录结构

```
control_plane/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI 应用入口
│   ├── config.py            # 配置管理 (Pydantic Settings)
│   ├── database.py          # 数据库连接
│   ├── dependencies.py      # FastAPI 依赖注入
│   ├── api/
│   │   └── v1/
│   │       ├── router.py    # 路由聚合
│   │       ├── auth.py      # 认证 API
│   │       ├── system.py    # 系统配置 API
│   │       ├── control.py   # 控制权管理 API
│   │       ├── mode.py      # 工作模式 API
│   │       ├── tasks.py     # 任务管理 API
│   │       ├── presets.py   # 预设管理 API
│   │       └── recording.py # 录制管理 API
│   ├── core/
│   │   ├── security.py      # JWT Token 处理
│   │   ├── control_lock.py  # 控制权互斥锁
│   │   └── mode_machine.py  # 模式状态机
│   ├── models/
│   │   ├── user.py          # 用户模型 (SQLAlchemy)
│   │   └── task.py          # 任务模型
│   ├── schemas/
│   │   ├── response.py      # 统一响应格式
│   │   ├── auth.py          # 认证相关 Schema
│   │   ├── system.py        # 系统配置 Schema
│   │   ├── control.py       # 控制权 Schema
│   │   ├── mode.py          # 工作模式 Schema
│   │   ├── task.py          # 任务 Schema
│   │   ├── preset.py        # 预设 Schema
│   │   └── recording.py     # 录制 Schema
│   └── services/
│       └── preset_manager.py # 预设管理服务
├── requirements.txt
└── pyproject.toml
```

## 环境变量

核心配置:
- SECRET_KEY 或 JWT_SECRET: JWT 密钥（生产必须设置）
- CORS_ORIGINS: 允许的前端域名列表
- WEBSOCKET_SERVER_URL: Data Plane 地址
- WEBRTC_SIGNALING_URL: Media Plane 地址

安全相关:
- AUTO_CREATE_ADMIN: 是否自动创建默认管理员
- DEFAULT_ADMIN_USERNAME: 默认管理员用户名
- DEFAULT_ADMIN_PASSWORD: 默认管理员密码（生产必须设置）
- VR_INTERNAL_TOKEN: VR 内部接口鉴权 Token
- TRUST_PROXY: 是否信任 X-Forwarded-For

说明:
- Data Plane 需配置同一个 Token（配置项 `control_sync.internal_token` 或环境变量 `CONTROL_PLANE_INTERNAL_TOKEN`）

## API 概览

### 认证 `/api/v1/auth`
| 方法 | 路径 | 描述 |
|------|------|------|
| POST | `/login` | 用户登录 |
| POST | `/logout` | 用户登出 |
| POST | `/refresh` | 刷新 Token |
| GET | `/me` | 获取当前用户信息 |

### 系统配置 `/api/v1/system`
| 方法 | 路径 | 描述 |
|------|------|------|
| GET | `/config` | 获取前端配置 |
| GET | `/health` | 健康检查 |
| GET | `/info` | 系统信息 |

### 控制权 `/api/v1/control`
| 方法 | 路径 | 描述 |
|------|------|------|
| POST | `/acquire` | 获取控制权 |
| POST | `/release` | 释放控制权 |
| POST | `/renew` | 续约控制权 |
| GET | `/status` | 查询控制权状态 |
| POST | `/force-release` | 强制释放（管理员） |

### 工作模式 `/api/v1/mode`
| 方法 | 路径 | 描述 |
|------|------|------|
| GET | `/current` | 获取当前模式 |
| POST | `/switch` | 切换模式 |
| GET | `/available` | 获取可用模式 |

### 任务管理 `/api/v1/tasks`
| 方法 | 路径 | 描述 |
|------|------|------|
| GET | `/` | 列出任务 |
| POST | `/` | 创建任务 |
| GET | `/{task_id}` | 获取任务详情 |
| PUT | `/{task_id}` | 更新任务 |
| DELETE | `/{task_id}` | 删除任务 |

### 预设管理 `/api/v1/presets`
| 方法 | 路径 | 描述 |
|------|------|------|
| GET | `/types` | 获取预设类型 |
| GET | `/` | 列出预设 |
| POST | `/` | 创建预设 |
| GET | `/{preset_type}/{preset_id}` | 获取预设 |
| PUT | `/{preset_type}/{preset_id}` | 更新预设 |
| DELETE | `/{preset_type}/{preset_id}` | 删除预设 |
| POST | `/{preset_type}/{preset_id}/apply` | 应用预设 |
| POST | `/capture` | 捕获当前状态为预设 |

### 录制管理 `/api/v1/recording`
| 方法 | 路径 | 描述 |
|------|------|------|
| POST | `/start` | 开始录制 |
| POST | `/stop` | 停止录制 |
| POST | `/discard` | 放弃录制 |
| GET | `/status` | 获取录制状态 |
| GET | `/files` | 获取录制文件列表 |
| DELETE | `/files/{file_id}` | 删除录制文件 |
| GET | `/topics` | 获取可用话题 |
| GET | `/topics/default` | 获取默认话题配置 |

### 紧急停止 `/api/v1/emergency` (HTTP 备用通道)
| 方法 | 路径 | 描述 |
|------|------|------|
| POST | `/stop` | 触发紧急停止 |
| GET | `/status` | 获取急停状态 |

> ⚠️ **注意**: 紧急停止的主通道是 WebSocket (`MSG_EMERGENCY_STOP`)，HTTP 接口作为备用方案，
> 用于 WebSocket 断开时的安全冗余（符合 ISO 10218 安全要求）。

### 导航控制 `/api/v1/chassis/navigate`
| 方法 | 路径 | 描述 |
|------|------|------|
| POST | `/pose` | 导航到坐标点 (x, y, yaw) |
| POST | `/station` | 导航到站点 (by ID 或 name) |
| POST | `/cancel` | 取消当前导航任务 |

> 💡 导航取消/暂停的推荐方式是 WebSocket (`MSG_NAVIGATION_CANCEL/PAUSE`)，延迟更低。

### 摄像头 `/api/v1/camera`
| 方法 | 路径 | 描述 |
|------|------|------|
| GET | `/list` | 获取可用摄像头列表 |
| GET | `/{camera_id}` | 获取指定摄像头信息 |
| GET | `/{camera_id}/webrtc` | 获取 WebRTC 连接信息 |

> 📹 实际视频流通过 Media Plane (WebRTC) 传输，本接口仅提供元数据。

### 底盘配置与状态 `/api/v1/chassis`
| 方法 | 路径 | 描述 |
|------|------|------|
| GET | `/config` | 获取底盘配置 |
| PUT | `/config` | 更新底盘配置 |
| POST | `/config/reset` | 重置底盘配置为默认值 |
| GET | `/status` | 获取底盘状态快照 |
| GET | `/stations` | 获取站点列表 |

> ⚠️ `/status` 接口仅用于页面初始化，**禁止高频轮询**！实时状态请订阅 Data Plane 的 `chassis_state`。

## API 请求/响应示例

### 紧急停止

```http
POST /api/v1/emergency/stop
Authorization: Bearer <token>
```

响应:
```json
{
  "success": true,
  "code": 0,
  "message": "紧急停止已触发",
  "data": {
    "emergency_active": true,
    "source": "http",
    "triggered_at": "2026-01-20T10:30:00Z"
  }
}
```

### 导航到站点

```http
POST /api/v1/chassis/navigate/station
Authorization: Bearer <token>
Content-Type: application/json

{
  "station_id": 1,
  "speed_factor": 0.8
}
```

响应:
```json
{
  "success": true,
  "code": 0,
  "message": "导航到站点 充电桩 已发起",
  "data": {
    "task_id": "a1b2c3d4",
    "station": {
      "id": 1,
      "name": "充电桩"
    },
    "target": {
      "x": 1.5,
      "y": 2.0,
      "yaw": 0.0
    }
  }
}
```

### 获取摄像头列表

```http
GET /api/v1/camera/list
Authorization: Bearer <token>
```

响应:
```json
{
  "success": true,
  "code": 0,
  "message": "获取摄像头列表成功",
  "data": {
    "cameras": [
      {
        "id": "head_rgb",
        "name": "头部 RGB 摄像头",
        "type": "rgb",
        "topic": "/head_camera/color/image_raw",
        "width": 1280,
        "height": 720,
        "fps": 30
      },
      {
        "id": "left_wrist",
        "name": "左手腕 RGB 摄像头",
        "type": "rgb",
        "topic": "/left_wrist_camera/color/image_raw",
        "width": 640,
        "height": 480,
        "fps": 30
      }
    ],
    "current_streaming": null
  }
}
```
| GET | `/{camera_id}/webrtc` | 获取 WebRTC 连接信息 |

> 📹 实际视频流通过 Media Plane (WebRTC) 传输，本接口仅提供元数据。

## 快速开始

### 安装依赖

```bash
cd control_plane
pip install -r requirements.txt
```

### 配置环境变量

创建 `.env` 文件：

```env
# 应用配置
DEBUG=true
HOST=0.0.0.0
PORT=8000

# 数据库
DATABASE_URL=sqlite:///~/qyh-robot-system/persistent/web/web.db

# JWT
JWT_SECRET_KEY=your-secret-key
JWT_ALGORITHM=HS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=60

# CORS
CORS_ORIGINS=["http://localhost:5173"]
```

### 运行服务

```bash
# 开发模式
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

# 生产模式
uvicorn app.main:app --host 0.0.0.0 --port 8000 --workers 4
```

### 访问文档

- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## 核心概念

### 统一响应格式

所有 API 返回统一的响应格式：

```json
{
  "success": true,
  "code": 0,
  "message": "操作成功",
  "data": { ... },
  "timestamp": "2024-01-01T00:00:00Z"
}
```

### 控制权互斥锁

同一时间只能有一个用户控制机器人，通过 `/api/v1/control` 管理：

1. 用户调用 `/acquire` 获取控制权
2. 控制权有有效期，需定期 `/renew` 续约
3. 使用完毕调用 `/release` 释放
4. 管理员可通过 `/force-release` 强制释放

### 工作模式状态机

机器人有多种工作模式，通过状态机管理切换：

- `idle`: 空闲
- `teleop`: 遥操作
- `auto`: 自动任务
- `recording`: 数据录制
- `training`: 模型训练

### 预设类型

支持以下预设类型：

| 类型 | 描述 |
|------|------|
| `arm_pose` | 手臂姿态 |
| `head_position` | 头部位置 |
| `lift_height` | 升降高度 |
| `waist_angle` | 腰部角度 |
| `location` | 导航点位 |
| `gripper_position` | 夹爪位置 |
| `full_pose` | 全身姿态 |

## 开发指南

### 添加新 API

1. 在 `schemas/` 创建请求/响应模型
2. 在 `api/v1/` 创建路由模块
3. 在 `api/v1/router.py` 注册路由
4. 在 `schemas/__init__.py` 导出模型

### 代码规范

- 使用 Pydantic V2 定义 Schema
- 使用 SQLAlchemy 2.0 风格
- API 返回统一使用 `success_response()` / `error_response()`
- 所有 API 需要 JWT 认证（除登录接口）

### ROS2 集成

部分功能需要 ROS2 服务支持（如预设应用、录制控制）。目前使用模拟实现，待集成 ROS2 后替换：

```python
# TODO: 实现实际的 ROS2 服务调用
async def call_ros2_service(...):
    # 使用 rclpy 或 ros2cli
    pass
```

## 待完成功能

参见 [TODO.md](./TODO.md) 了解详细的实现状态和待办事项。

### P0 优先级（基础功能）
- [x] 认证系统
- [x] 系统配置
- [x] 控制权管理
- [x] 工作模式
- [x] 任务管理（基础 CRUD）
- [x] 预设管理
- [x] 录制管理

### P1 优先级（增强功能）
- [ ] ROS2 服务集成
- [ ] 动作管理 API
- [ ] 机器人信息 API
- [ ] 系统关机 API
- [ ] 审计日志

## License

内部使用
