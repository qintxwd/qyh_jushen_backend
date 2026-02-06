# QYH Jushen Backend API 接口文档

> **版本**: 1.0.1  
> **更新日期**: 2026-02-06  
> **基础路径**: `/api/v1`
---

## 目录

1. [架构概述](#1-架构概述)
  - [1.1 接口职责划分原则](#11-接口职责划分原则)
  - [1.2 服务发现](#12-服务发现)
  - [1.3 编译与启动](#13-编译与启动)
2. [通用约定](#2-通用约定)
3. [Control Plane - HTTP REST API](#3-control-plane---http-rest-api)
   - [3.1 认证 (Auth)](#31-认证-auth)
   - [3.2 系统配置 (System)](#32-系统配置-system)
   - [3.3 控制权管理 (Control)](#33-控制权管理-control)
   - [3.4 紧急停止 (Emergency)](#34-紧急停止-emergency)
   - [3.5 工作模式 (Mode)](#35-工作模式-mode)
   - [3.6 任务管理 (Tasks)](#36-任务管理-tasks)
   - [3.7 预设管理 (Presets)](#37-预设管理-presets)
   - [3.8 录制管理 (Recording)](#38-录制管理-recording)
   - [3.9 动作管理 (Actions)](#39-动作管理-actions)
   - [3.10 底盘配置 (Chassis)](#310-底盘配置-chassis)
   - [3.11 机械臂控制 (Arm)](#311-机械臂控制-arm)
   - [3.12 摄像头 (Camera)](#312-摄像头-camera)
   - [3.13 LED控制 (LED)](#313-led控制-led)
   - [3.14 VR状态 (VR)](#314-vr状态-vr)
   - [3.15 机器人信息 (Robot)](#315-机器人信息-robot)
   - [3.16 审计日志 (Audit)](#316-审计日志-audit)
4. [Data Plane - WebSocket API](#4-data-plane---websocket-api)
5. [Media Plane - WebRTC API](#5-media-plane---webrtc-api)
6. [错误码参考](#6-错误码参考)

---

## 1. 架构概述

本系统采用**三平面分离架构**，每个平面负责不同类型的通信：

| 平面 | 协议 | 端口 | 职责 | 延迟要求 |
|------|------|------|------|----------|
| **Control Plane** | HTTP/REST | 8000 | 认证、配置、CRUD 操作 | 100-500ms |
| **Data Plane** | WebSocket + Protobuf | 8765 | 实时控制、状态推送 | <20ms |
| **Media Plane** | WebRTC | 8888 | 视频流传输 | <100ms |

### 1.1 接口职责划分原则

| 操作类型 | 频率 | 协议 | 示例 |
|----------|------|------|------|
| 配置管理 | <5Hz | HTTP | 用户认证、任务CRUD、预设管理 |
| 实时控制 | 30-100Hz | WebSocket | 底盘速度、关节控制、急停 |
| 视频传输 | 30fps | WebRTC | 摄像头视频流 |

### 1.2 服务发现

前端只需硬编码 Control Plane 地址，其他服务地址通过 `GET /api/v1/system/config` 动态获取。

### 1.3 编译与启动

使用一条命令编译部署，再用启动脚本拉起服务：

```bash
./build_all.sh
./start.sh
```

---

## 2. 通用约定

### 2.1 认证方式

所有需要认证的接口使用 **JWT Bearer Token**：

```http
Authorization: Bearer <access_token>
```

### 2.2 统一响应格式

**成功响应**:
```json
{
  "success": true,
  "code": 0,
  "message": "操作成功",
  "data": { ... },
  "timestamp": "2026-02-05T10:30:00Z"
}
```

**错误响应**:
```json
{
  "success": false,
  "code": 40001,
  "message": "错误描述",
  "data": null,
  "timestamp": "2026-02-05T10:30:00Z"
}
```

**分页响应**:
```json
{
  "success": true,
  "code": 0,
  "message": "获取成功",
  "data": {
    "items": [...],
    "total": 100,
    "page": 1,
    "page_size": 20
  }
}
```

### 2.3 权限角色

| 角色 | 说明 | 权限 |
|------|------|------|
| `admin` | 管理员 | 所有操作 |
| `operator` | 操作员 | 控制操作、任务管理 |
| `viewer` | 观察者 | 只读访问 |

---

## 3. Control Plane - HTTP REST API

### 3.1 认证 (Auth)

#### 3.1.1 用户登录

```http
POST /api/v1/auth/login
```

**请求体**:
```json
{
  "username": "admin",
  "password": "password123"
}
```

**响应**:
```json
{
  "success": true,
  "data": {
    "access_token": "eyJhbGciOiJIUzI1NiIs...",
    "token_type": "bearer",
    "expires_in": 86400,
    "user": {
      "id": 1,
      "username": "admin",
      "email": "admin@example.com",
      "role": "admin",
      "is_active": true
    }
  },
  "message": "登录成功"
}
```

#### 3.1.2 用户登出

```http
POST /api/v1/auth/logout
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "message": "登出成功"
}
```

#### 3.1.3 获取当前用户信息

```http
GET /api/v1/auth/me
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "id": 1,
    "username": "admin",
    "email": "admin@example.com",
    "role": "admin",
    "is_active": true
  }
}
```

#### 3.1.4 刷新 Token

```http
POST /api/v1/auth/refresh
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "refreshed": true,
    "access_token": "eyJhbGciOiJIUzI1NiIs...",
    "token_type": "bearer",
    "expires_in": 86400
  }
}
```

---

### 3.2 系统配置 (System)

#### 3.2.1 获取系统配置 (服务发现)

```http
GET /api/v1/system/config
```

**响应**:
```json
{
  "success": true,
  "data": {
    "robot_id": "qyh-jushen-001",
    "robot_name": "QYH Jushen",
    "endpoints": {
      "websocket": "ws://192.168.1.100:8765",
      "webrtc_signaling": "ws://192.168.1.100:8888"
    },
    "webrtc": {
      "ice_servers": [
        { "urls": "stun:stun.l.google.com:19302" }
      ]
    },
    "features": {
      "vr_teleop": true,
      "multi_camera": true,
      "recording": true,
      "navigation": true
    }
  }
}
```

#### 3.2.2 获取系统健康状态

```http
GET /api/v1/system/health
```

**响应**:
```json
{
  "success": true,
  "data": {
    "overall_status": "healthy",
    "services": [
      {
        "name": "control_plane",
        "status": "healthy",
        "latency_ms": 0.1,
        "message": "Control Plane 运行正常"
      },
      {
        "name": "data_plane",
        "status": "healthy",
        "latency_ms": 5.2,
        "message": "WebSocket 服务正常"
      },
      {
        "name": "media_plane",
        "status": "healthy",
        "latency_ms": 8.1,
        "message": "WebRTC 服务正常"
      },
      {
        "name": "ros2",
        "status": "healthy",
        "latency_ms": 2.3,
        "message": "ROS2 连接正常"
      }
    ],
    "ros2_connected": true,
    "robot_mode": "idle"
  }
}
```

#### 3.2.3 获取系统信息

```http
GET /api/v1/system/info
```

**响应**:
```json
{
  "success": true,
  "data": {
    "app_name": "QYH Jushen Backend",
    "app_version": "1.0.0",
    "environment": "production",
    "robot_id": "qyh-jushen-001",
    "robot_name": "QYH Jushen",
    "uptime_seconds": 86400,
    "ros2_domain_id": 42
  }
}
```

---

### 3.3 控制权管理 (Control)

#### 3.3.1 获取控制权

```http
POST /api/v1/control/acquire
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "duration": 300,
  "session_type": "teleop"
}
```

| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| duration | int | 否 | 控制权持续时间（秒），默认 300 |
| session_type | string | 否 | 会话类型: `teleop`, `auto`, `vr` |

**响应**:
```json
{
  "success": true,
  "data": {
    "holder": {
      "user_id": 1,
      "username": "admin",
      "acquired_at": "2026-02-05T10:30:00Z",
      "expires_at": "2026-02-05T10:35:00Z",
      "session_type": "teleop"
    }
  },
  "message": "控制权获取成功"
}
```

**错误响应** (已被占用):
```json
{
  "success": false,
  "code": 40301,
  "message": "控制权已被用户 operator1 持有",
  "data": {
    "holder": {
      "user_id": 2,
      "username": "operator1",
      "acquired_at": "2026-02-05T10:25:00Z",
      "expires_at": "2026-02-05T10:30:00Z",
      "session_type": "teleop"
    }
  }
}
```

#### 3.3.2 释放控制权

```http
POST /api/v1/control/release
```

**需要认证**: 是 (operator+)

**响应**:
```json
{
  "success": true,
  "message": "控制权已释放"
}
```

#### 3.3.3 续约控制权

```http
POST /api/v1/control/renew
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "duration": 300,
  "session_type": "teleop"
}
```

#### 3.3.4 查询控制权状态

```http
GET /api/v1/control/status
```

**需要认证**: 否

**响应**:
```json
{
  "success": true,
  "data": {
    "locked": true,
    "holder": {
      "user_id": 1,
      "username": "admin",
      "acquired_at": "2026-02-05T10:30:00Z",
      "expires_at": "2026-02-05T10:35:00Z",
      "session_type": "teleop"
    }
  }
}
```

#### 3.3.5 强制释放控制权 (管理员)

```http
POST /api/v1/control/force-release
```

**需要认证**: 是 (admin)

**请求体**:
```json
{
  "reason": "紧急情况需要接管",
  "session_id": "optional-session-id"
}
```

#### 3.3.6 获取控制权历史

```http
GET /api/v1/control/history
```

**需要认证**: 是 (operator+)

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| user_id | int | 按用户过滤（管理员可查看所有） |
| limit | int | 返回数量限制，默认 50 |
| offset | int | 偏移量 |

#### 3.3.7 获取控制权统计 (管理员)

```http
GET /api/v1/control/statistics
```

**需要认证**: 是 (admin)

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| days | int | 统计天数，默认 7 |

---

### 3.4 紧急停止 (Emergency)

> ⚠️ **安全关键功能** - 双通道冗余设计
> - **主通道**: WebSocket `MSG_EMERGENCY_STOP` (延迟 <20ms)
> - **备用通道**: 本 HTTP 接口 (延迟 ~100ms)

#### 3.4.1 触发紧急停止

```http
POST /api/v1/emergency/stop
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "stopped": true,
    "chassis": true,
    "arm": true,
    "actuators": true,
    "timestamp": "2026-02-05T10:30:00Z"
  },
  "message": "紧急停止已执行，所有运动部件已停止"
}
```

#### 3.4.2 获取急停状态

```http
GET /api/v1/emergency/status
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "in_estop": false,
    "can_resume": true
  }
}
```

---

### 3.5 工作模式 (Mode)

#### 3.5.1 获取当前模式

```http
GET /api/v1/mode/current
```

**响应**:
```json
{
  "success": true,
  "data": {
    "mode": "idle",
    "since": "2026-02-05T10:00:00Z",
    "can_switch_to": ["teleop", "auto", "maintenance"]
  }
}
```

**模式说明**:
| 模式 | 说明 |
|------|------|
| `idle` | 空闲状态 |
| `teleop` | 遥操作模式 |
| `auto` | 自主任务模式 |
| `maintenance` | 维护模式 |
| `error` | 错误状态 |

#### 3.5.2 切换模式

```http
POST /api/v1/mode/switch
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "target_mode": "teleop",
  "force": false
}
```

**模式切换规则**:
- `idle` → `teleop`, `auto`, `maintenance`
- `teleop` → `idle` (需先释放控制权)
- `auto` → `idle` (任务完成或取消后)
- `maintenance` → `idle`
- `error` → `idle`, `maintenance`

#### 3.5.3 获取可用模式列表

```http
GET /api/v1/mode/available
```

**响应**:
```json
{
  "success": true,
  "data": {
    "modes": [
      { "mode": "idle", "name": "空闲", "description": "机器人处于空闲状态" },
      { "mode": "teleop", "name": "遥操作", "description": "人工遥控模式" },
      { "mode": "auto", "name": "自主", "description": "执行自动任务" },
      { "mode": "maintenance", "name": "维护", "description": "维护调试模式" }
    ]
  }
}
```

---

### 3.6 任务管理 (Tasks)

#### 3.6.1 获取任务列表

```http
GET /api/v1/tasks
```

**需要认证**: 是

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| status_filter | string | 状态过滤: `pending`, `running`, `paused`, `completed`, `failed`, `cancelled` |
| page | int | 页码，默认 1 |
| page_size | int | 每页数量，默认 20 |

**响应**:
```json
{
  "success": true,
  "data": {
    "items": [
      {
        "task_id": 1,
        "name": "巡检任务A",
        "description": "执行A区域巡检",
        "status": "pending",
        "current_step": 0,
        "total_steps": 5,
        "progress": 0.0,
        "created_at": "2026-02-05T10:00:00Z",
        "started_at": null,
        "completed_at": null,
        "error_message": null
      }
    ],
    "total": 10,
    "page": 1,
    "page_size": 20
  }
}
```

#### 3.6.2 创建任务

```http
POST /api/v1/tasks
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "name": "巡检任务A",
  "description": "执行A区域巡检",
  "program": [
    { "type": "navigate", "station_id": 1 },
    { "type": "wait", "duration": 5 },
    { "type": "navigate", "station_id": 2 },
    { "type": "preset", "preset_type": "arm_pose", "preset_id": "home" }
  ]
}
```

#### 3.6.3 获取任务详情

```http
GET /api/v1/tasks/{task_id}
```

#### 3.6.4 启动任务

```http
POST /api/v1/tasks/{task_id}/start
```

**需要认证**: 是 (operator+)

#### 3.6.5 暂停任务

```http
POST /api/v1/tasks/{task_id}/pause
```

**需要认证**: 是 (operator+)

#### 3.6.6 恢复任务

```http
POST /api/v1/tasks/{task_id}/resume
```

**需要认证**: 是 (operator+)

#### 3.6.7 取消任务

```http
POST /api/v1/tasks/{task_id}/cancel
```

**需要认证**: 是 (operator+)

#### 3.6.8 删除任务

```http
DELETE /api/v1/tasks/{task_id}
```

**需要认证**: 是 (operator+)

---

### 3.7 预设管理 (Presets)

#### 3.7.1 获取预设类型列表

```http
GET /api/v1/presets/types
```

**响应**:
```json
{
  "success": true,
  "data": {
    "types": [
      {
        "type": "arm_pose",
        "name": "机械臂点位",
        "description": "机械臂关节角度或末端位姿预设",
        "data_fields": ["side", "pose_type", "left_joints", "right_joints", "velocity", "acceleration"]
      },
      {
        "type": "head_position",
        "name": "头部点位",
        "description": "头部云台角度预设",
        "data_fields": ["pan", "tilt"]
      },
      {
        "type": "lift_height",
        "name": "升降高度",
        "description": "升降柱高度预设",
        "data_fields": ["height"]
      },
      {
        "type": "waist_angle",
        "name": "腰部角度",
        "description": "腰部前倾角度预设",
        "data_fields": ["angle"]
      },
      {
        "type": "location",
        "name": "导航点位",
        "description": "底盘导航目标点预设",
        "data_fields": ["x", "y", "theta", "frame_id", "station_id"]
      },
      {
        "type": "gripper_position",
        "name": "夹爪位置",
        "description": "夹爪开合位置预设",
        "data_fields": ["side", "left_position", "right_position", "force"]
      },
      {
        "type": "full_pose",
        "name": "完整姿态",
        "description": "机器人完整姿态预设（多部件组合）",
        "data_fields": ["arm", "head", "lift", "waist", "gripper"]
      }
    ]
  }
}
```

#### 3.7.2 获取预设列表

```http
GET /api/v1/presets
```

**查询参数**:
| 参数 | 类型 | 必填 | 说明 |
|------|------|------|------|
| preset_type | string | 是 | 预设类型 |
| category | string | 否 | 分类过滤 |
| include_builtin | bool | 否 | 是否包含内置预设，默认 true |

**响应**:
```json
{
  "success": true,
  "data": {
    "preset_type": "arm_pose",
    "total": 5,
    "items": [
      {
        "id": "home",
        "name": "初始位置",
        "description": "机械臂初始姿态",
        "is_builtin": true,
        "category": "basic",
        "data": {
          "side": "both",
          "left_joints": [0, 0, 0, 0, 0, 0, 0],
          "right_joints": [0, 0, 0, 0, 0, 0, 0]
        }
      }
    ]
  }
}
```

#### 3.7.3 创建预设

```http
POST /api/v1/presets
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "preset_type": "arm_pose",
  "name": "抓取位置1",
  "description": "抓取工位1的姿态",
  "category": "grab",
  "data": {
    "side": "both",
    "left_joints": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
    "right_joints": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
    "velocity": 0.5,
    "acceleration": 0.3
  }
}
```

#### 3.7.4 获取预设详情

```http
GET /api/v1/presets/{preset_type}/{preset_id}
```

#### 3.7.5 更新预设

```http
PUT /api/v1/presets/{preset_type}/{preset_id}
```

**需要认证**: 是 (operator+)

#### 3.7.6 删除预设

```http
DELETE /api/v1/presets/{preset_type}/{preset_id}
```

**需要认证**: 是 (operator+)

> 注意：内置预设不可删除

#### 3.7.7 应用预设

```http
POST /api/v1/presets/{preset_type}/{preset_id}/apply
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "velocity_scale": 0.8,
  "wait_completion": true
}
```

---

### 3.8 录制管理 (Recording)

#### 3.8.1 开始录制

```http
POST /api/v1/recording/start
```

**需要认证**: 是 (operator+) + 需要持有控制锁

**请求体**:
```json
{
  "action_name": "pickup_cube",
  "user_name": "operator1",
  "version": "1.0",
  "topics": [
    "/head_camera/color/image_raw",
    "/left_wrist_camera/color/image_raw",
    "/joint_states",
    "/left_gripper/state"
  ]
}
```

**响应**:
```json
{
  "success": true,
  "data": {
    "success": true,
    "message": "录制已开始",
    "bag_path": "/home/user/qyh-robot-system/model_actions/pickup_cube/data/bags/qyh_operator1_1.0_20260205_103000"
  }
}
```

#### 3.8.2 停止录制

```http
POST /api/v1/recording/stop
```

**需要认证**: 是 (operator+) + 需要持有控制锁

**响应**:
```json
{
  "success": true,
  "data": {
    "success": true,
    "bag_path": "/home/user/qyh-robot-system/model_actions/pickup_cube/data/bags/qyh_operator1_1.0_20260205_103000",
    "duration_seconds": 30.5
  }
}
```

#### 3.8.3 放弃录制

```http
POST /api/v1/recording/discard
```

**需要认证**: 是 (operator+) + 需要持有控制锁

#### 3.8.4 获取录制状态

```http
GET /api/v1/recording/status
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "is_recording": true,
    "status": "recording",
    "action_name": "pickup_cube",
    "user_name": "operator1",
    "version": "1.0",
    "duration_seconds": 15.3,
    "bag_path": "/home/user/...",
    "topics": ["..."],
    "started_at": "2026-02-05T10:30:00Z"
  }
}
```

#### 3.8.5 获取可用话题

```http
GET /api/v1/recording/topics
```

**需要认证**: 是

#### 3.8.6 获取默认话题

```http
GET /api/v1/recording/default-topics
```

#### 3.8.7 获取录制文件列表

```http
GET /api/v1/recording/files
```

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| action_name | string | 按动作名称过滤 |

---

### 3.9 动作管理 (Actions)

**命名约束**:
- `action_id` 仅允许小写字母、数字、下划线和短横线
- 正则: `^[a-z0-9][a-z0-9_-]{0,63}$`

#### 3.9.1 获取机器人信息

```http
GET /api/v1/actions/robot-info
```

**响应**:
```json
{
  "success": true,
  "data": {
    "robot_name": "general",
    "robot_version": "1.0",
    "actions_dir": "/home/user/qyh-robot-system/model_actions/general/1.0"
  }
}
```

#### 3.9.2 获取动作列表

```http
GET /api/v1/actions
```

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| status | string | 状态过滤: `collecting`, `trained` |
| robot_name | string | 机器人类型 |
| robot_version | string | 机器人版本 |

**响应**:
```json
{
  "success": true,
  "data": {
    "robot_name": "general",
    "robot_version": "1.0",
    "total": 2,
    "items": [
      {
        "id": "pickup_cube",
        "name": "抓取方块",
        "description": "从桌面抓取方块的动作",
        "version": "1.0.0",
        "tags": ["manipulation", "pickup"],
        "status": "trained",
        "has_model": true,
        "episode_count": 50,
        "model_version": 3,
        "topics": ["/head_camera/color/image_raw", "/joint_states"],
        "camera_count": 2,
        "created_at": "2026-01-15T10:00:00Z",
        "updated_at": "2026-02-01T15:30:00Z"
      }
    ]
  }
}
```

#### 3.9.3 获取已训练动作列表

```http
GET /api/v1/actions/trained
```

#### 3.9.4 获取采集中动作列表

```http
GET /api/v1/actions/collecting
```

#### 3.9.5 创建动作

```http
POST /api/v1/actions
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "id": "place_object",
  "name": "放置物品",
  "description": "将物品放置到指定位置",
  "template": "pickup_cube"
}
```

#### 3.9.6 获取动作详情

```http
GET /api/v1/actions/{action_id}
```

#### 3.9.7 更新动作

```http
PUT /api/v1/actions/{action_id}
```

**需要认证**: 是 (operator+)

#### 3.9.8 删除动作

```http
DELETE /api/v1/actions/{action_id}
```

**需要认证**: 是 (operator+)

#### 3.9.9 标记为已训练

```http
POST /api/v1/actions/{action_id}/mark-trained
```

**需要认证**: 是 (operator+)

#### 3.9.10 获取轨迹列表

```http
GET /api/v1/actions/{action_id}/episodes
```

#### 3.9.11 获取推理配置

```http
GET /api/v1/actions/{action_id}/inference-config
```

---

### 3.10 底盘配置 (Chassis)

#### 3.10.1 获取底盘配置

```http
GET /api/v1/chassis/config
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "speed_level": 50,
    "volume": 50,
    "obstacle_strategy": 1
  }
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| speed_level | int | 速度级别 (1-100) |
| volume | int | 音量 (0-100) |
| obstacle_strategy | int | 避障策略: 0=禁用, 1=正常, 2=激进 |

#### 3.10.2 更新底盘配置

```http
PUT /api/v1/chassis/config
```

**需要认证**: 是 (operator+)

**请求体** (支持部分更新):
```json
{
  "speed_level": 70,
  "volume": 30
}
```

#### 3.10.3 重置底盘配置

```http
POST /api/v1/chassis/config/reset
```

**需要认证**: 是 (operator+)

#### 3.10.4 获取底盘状态

```http
GET /api/v1/chassis/status
```

**需要认证**: 是

> ⚠️ 此接口用于页面初始化，不要高频轮询！实时状态请订阅 WebSocket。

**响应**:
```json
{
  "success": true,
  "data": {
    "connected": true,
    "system_status": 1,
    "system_status_text": "正常",
    "location_status": 2,
    "location_status_text": "已定位",
    "operation_status": 0,
    "operation_status_text": "空闲",
    "pose": {
      "x": 1.5,
      "y": 2.3,
      "yaw": 0.785,
      "confidence": 1.0
    },
    "velocity": {
      "linear_x": 0.0,
      "linear_y": 0.0,
      "angular_z": 0.0
    },
    "battery": {
      "percentage": 85,
      "voltage": 24.5,
      "current": 2.1,
      "status_text": "放电中"
    },
    "flags": {
      "is_emergency_stopped": false,
      "is_charging": false,
      "is_auto_mode": false
    }
  }
}
```

#### 3.10.5 获取站点列表

```http
GET /api/v1/chassis/stations
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "stations": [
      { "id": 1, "name": "充电站", "x": 0.0, "y": 0.0, "yaw": 0.0 },
      { "id": 2, "name": "工位A", "x": 5.0, "y": 3.0, "yaw": 1.57 }
    ]
  }
}
```

#### 3.10.6 导航到坐标点

```http
POST /api/v1/chassis/navigate/pose
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "x": 5.0,
  "y": 3.0,
  "yaw": 1.57,
  "speed_factor": 0.8
}
```

#### 3.10.7 导航到站点

```http
POST /api/v1/chassis/navigate/station
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "station_id": 2,
  "speed_factor": 0.8
}
```

或:
```json
{
  "station_name": "工位A",
  "speed_factor": 0.8
}
```

#### 3.10.8 取消导航

```http
POST /api/v1/chassis/navigate/cancel
```

**需要认证**: 是 (operator+)

> 💡 推荐使用 WebSocket 取消导航以获得更低延迟

#### 3.10.9 获取地图数据

```http
GET /api/v1/chassis/map_data
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "success": true,
    "map_name": "standard",
    "meta": { ... },
    "nodes": [ ... ],
    "edges": [ ... ],
    "stations": [ ... ],
    "has_image": true,
    "image_url": "/api/v1/chassis/map_image/standard"
  }
}
```

#### 3.10.10 获取地图图片

```http
GET /api/v1/chassis/map_image/{map_name}
```

**需要认证**: 否

**响应**: PNG/JPEG 图片

#### 3.10.11 获取地图列表

```http
GET /api/v1/chassis/maps
```

---

### 3.11 机械臂控制 (Arm)

> ⚠️ **接口职责分离**
> - ✅ 本模块: 低频配置操作 (电源、使能、负载配置、点位CRUD)
> - ❌ 高频控制: 必须走 WebSocket (`MSG_ARM_JOG`, `MSG_ARM_MOVE`, `MSG_JOINT_COMMAND`)

#### 3.11.1 获取机械臂状态

```http
GET /api/v1/arm/state
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "connected": true,
    "robot_ip": "192.168.1.50",
    "powered_on": true,
    "enabled": true,
    "in_estop": false,
    "in_error": false,
    "servo_mode_enabled": true,
    "error_message": ""
  }
}
```

#### 3.11.2 获取伺服状态

```http
GET /api/v1/arm/servo/status
```

#### 3.11.3 上电

```http
POST /api/v1/arm/power_on
```

**需要认证**: 是 (operator+)

#### 3.11.4 下电

```http
POST /api/v1/arm/power_off
```

**需要认证**: 是 (operator+)

#### 3.11.5 使能

```http
POST /api/v1/arm/enable
```

**需要认证**: 是 (operator+)

#### 3.11.6 去使能

```http
POST /api/v1/arm/disable
```

**需要认证**: 是 (operator+)

#### 3.11.7 清除错误

```http
POST /api/v1/arm/clear_error
```

**需要认证**: 是 (operator+)

#### 3.11.8 急停 (HTTP备用)

```http
POST /api/v1/arm/motion_abort
```

**需要认证**: 是 (operator+)

> ⚠️ 主通道是 WebSocket 的 `MSG_EMERGENCY_STOP`

#### 3.11.9 启动伺服模式

```http
POST /api/v1/arm/servo/start
```

**需要认证**: 是 (operator+)

#### 3.11.10 停止伺服模式

```http
POST /api/v1/arm/servo/stop
```

**需要认证**: 是 (operator+)

#### 3.11.11 获取夹爪负载配置

```http
GET /api/v1/arm/payload/gripper_config
```

**响应**:
```json
{
  "success": true,
  "data": {
    "left_gripper_mass": 0.8,
    "right_gripper_mass": 0.8
  }
}
```

#### 3.11.12 设置夹爪负载配置

```http
POST /api/v1/arm/payload/gripper_config
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "left_gripper_mass": 0.9,
  "right_gripper_mass": 0.85
}
```

#### 3.11.13 应用夹爪负载

```http
POST /api/v1/arm/payload/apply_gripper
```

**需要认证**: 是 (operator+)

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| robot_id | int | 0=左臂, 1=右臂 |

#### 3.11.14 设置物品负载

```http
POST /api/v1/arm/payload/set_object
```

**需要认证**: 是 (operator+)

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| robot_id | int | 0=左臂, 1=右臂 |
| object_mass | float | 物品重量 (0-5 kg) |

#### 3.11.15 获取负载状态

```http
GET /api/v1/arm/payload/status
```

#### 3.11.16 获取机械臂点位列表

```http
GET /api/v1/arm/points
```

#### 3.11.17 创建机械臂点位

```http
POST /api/v1/arm/points
```

**需要认证**: 是 (operator+)

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| name | string | 点位名称 |
| description | string | 点位描述 |
| side | string | 采集侧: `left`, `right`, `both` |

---

### 3.12 摄像头 (Camera)

#### 3.12.1 获取摄像头列表

```http
GET /api/v1/camera/list
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "cameras": [
      {
        "id": "head_rgb",
        "name": "头部 RGB 摄像头",
        "type": "rgb",
        "topic": "/head_camera/color/image_raw",
        "width": 1280,
        "height": 720,
        "fps": 30,
        "encoding": "bgr8",
        "webrtc_track_id": null
      },
      {
        "id": "head_depth",
        "name": "头部深度摄像头",
        "type": "depth",
        "topic": "/head_camera/depth/image_rect_raw",
        "width": 640,
        "height": 480,
        "fps": 30,
        "encoding": "16UC1"
      },
      {
        "id": "left_wrist",
        "name": "左手腕 RGB 摄像头",
        "type": "rgb",
        "topic": "/left_wrist_camera/color/image_raw",
        "width": 640,
        "height": 480,
        "fps": 30,
        "encoding": "bgr8"
      },
      {
        "id": "right_wrist",
        "name": "右手腕 RGB 摄像头",
        "type": "rgb",
        "topic": "/right_wrist_camera/color/image_raw",
        "width": 640,
        "height": 480,
        "fps": 30,
        "encoding": "bgr8"
      }
    ],
    "current_streaming": null
  }
}
```

#### 3.12.2 获取摄像头信息

```http
GET /api/v1/camera/{camera_id}
```

#### 3.12.3 获取 WebRTC 连接信息

```http
GET /api/v1/camera/{camera_id}/webrtc
```

> `signaling_url` 由系统配置 `WEBRTC_SIGNALING_URL` 推导

---

### 3.13 LED控制 (LED)

#### 3.13.1 获取LED状态

```http
GET /api/v1/led/state
```

**需要认证**: 是 (operator+)

**响应**:
```json
{
  "success": true,
  "data": {
    "is_blinking": false,
    "current_color": { "r": 0, "g": 255, "b": 0, "w": 0 },
    "blink_colors": null,
    "blink_interval_ms": null
  }
}
```

#### 3.13.2 设置LED颜色

```http
POST /api/v1/led/color
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "r": 255,
  "g": 128,
  "b": 0,
  "w": 0
}
```

#### 3.13.3 设置预设颜色

```http
POST /api/v1/led/preset/{preset_name}
```

**预设颜色**:
- `red`, `green`, `blue`, `white`, `yellow`, `cyan`, `magenta`, `orange`, `purple`, `warm_white`, `off`

#### 3.13.4 设置闪烁模式

```http
POST /api/v1/led/blink
```

**需要认证**: 是 (operator+)

**请求体**:
```json
{
  "colors": [
    { "r": 255, "g": 0, "b": 0, "w": 0 },
    { "r": 0, "g": 0, "b": 0, "w": 0 }
  ],
  "interval_ms": 500
}
```

#### 3.13.5 设置预设闪烁模式

```http
POST /api/v1/led/blink/preset/{mode}
```

**预设模式**:
- `warning` - 黄色警告闪烁
- `error` - 红色错误闪烁
- `success` - 绿色成功闪烁
- `processing` - 蓝色处理中
- `rainbow` - 彩虹循环
- `police` - 红蓝交替

#### 3.13.6 停止闪烁

```http
POST /api/v1/led/blink/stop
```

---

### 3.14 VR状态 (VR)

#### 3.14.1 获取VR连接状态

```http
GET /api/v1/vr/status
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "connected": true,
    "client_info": {
      "device": "PICO 4",
      "version": "1.0.0",
      "session_id": "abc123",
      "connected_at": "2026-02-05T10:30:00Z"
    }
  }
}
```

#### 3.14.2 VR连接上报 (内部接口)

```http
POST /api/v1/vr/internal/connected
```

> 由 Data Plane 在 VR 客户端连接时调用
> 可选鉴权: 如果配置了 `VR_INTERNAL_TOKEN`，需携带 `Authorization: Bearer <token>`

#### 3.14.3 VR断开上报 (内部接口)

```http
POST /api/v1/vr/internal/disconnected
```

> 由 Data Plane 在 VR 客户端断开时调用
> 可选鉴权: 如果配置了 `VR_INTERNAL_TOKEN`，需携带 `Authorization: Bearer <token>`

---

### 3.15 机器人信息 (Robot)

#### 3.15.1 获取机器人信息

```http
GET /api/v1/robot/info
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "name": "QYH Jushen",
    "model": "general",
    "version": "1.0",
    "serial_number": "QYH-001",
    "has_left_arm": true,
    "has_right_arm": true,
    "has_head": true,
    "has_lift": true,
    "has_waist": true,
    "has_chassis": true,
    "left_arm_joints": 7,
    "right_arm_joints": 7,
    "head_joints": 2,
    "urdf_path": "/home/user/qyh-robot-system/..."
  }
}
```

#### 3.15.2 获取状态概览

```http
GET /api/v1/robot/overview
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "timestamp": "2026-02-05T10:30:00Z",
    "name": "QYH Jushen",
    "model": "general",
    "version": "1.0",
    "subsystems": [
      { "name": "ROS2 Core", "connected": true, "status": "ok", "message": "正常" },
      { "name": "Data Plane", "connected": true, "status": "ok", "message": "WebSocket 服务正常" },
      { "name": "Media Plane", "connected": true, "status": "ok", "message": "WebRTC 服务正常" },
      { "name": "Left Arm", "connected": true, "status": "ok", "message": "正常" },
      { "name": "Right Arm", "connected": true, "status": "ok", "message": "正常" },
      { "name": "Head", "connected": true, "status": "ok", "message": "正常" },
      { "name": "Chassis", "connected": true, "status": "ok", "message": "正常" },
      { "name": "Lift", "connected": true, "status": "ok", "message": "正常" },
      { "name": "Waist", "connected": true, "status": "ok", "message": "正常" }
    ],
    "system": {
      "cpu_temp": 45.2,
      "gpu_temp": 52.1,
      "battery": 85.0,
      "mode": "idle",
      "uptime_seconds": 86400
    }
  }
}
```

#### 3.15.3 获取URDF模型

```http
GET /api/v1/robot/urdf
```

**需要认证**: 是

**响应**: XML 格式的 URDF 文件

#### 3.15.4 获取关机状态

```http
GET /api/v1/robot/shutdown/state
```

**需要认证**: 是

**响应**:
```json
{
  "success": true,
  "data": {
    "shutdown_in_progress": false,
    "trigger_source": 0,
    "trigger_source_text": "",
    "countdown_seconds": -1,
    "plc_connected": true
  }
}
```

#### 3.15.5 系统关机 (管理员)

```http
POST /api/v1/robot/shutdown
```

**需要认证**: 是 (admin)

**请求体**:
```json
{
  "reason": "计划维护",
  "delay_seconds": 10
}
```

#### 3.15.6 系统重启 (管理员)

```http
POST /api/v1/robot/reboot
```

**需要认证**: 是 (admin)

#### 3.15.7 获取ROS包文件

```http
GET /api/v1/robot/package/{package_name}/{file_path}
```

> 用于获取 URDF 引用的 mesh 等资源文件

---

### 3.16 审计日志 (Audit)

#### 3.16.1 查询审计日志

```http
GET /api/v1/audit
```

**需要认证**: 是

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| user_id | int | 用户ID |
| username | string | 用户名（模糊匹配） |
| action | string | 操作类型 |
| resource | string | 资源类型 |
| resource_id | string | 资源ID |
| start_time | datetime | 开始时间 |
| end_time | datetime | 结束时间 |
| ip_address | string | IP地址 |
| page | int | 页码 |
| page_size | int | 每页数量 (1-100) |

> 非管理员只能查看自己的日志

#### 3.16.2 获取操作类型列表

```http
GET /api/v1/audit/actions
```

#### 3.16.3 获取资源类型列表

```http
GET /api/v1/audit/resources
```

#### 3.16.4 获取审计统计 (管理员)

```http
GET /api/v1/audit/statistics
```

**需要认证**: 是 (admin)

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| days | int | 统计天数 (1-90)，默认 7 |

#### 3.16.5 获取我的最近操作

```http
GET /api/v1/audit/my-recent
```

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| limit | int | 数量限制 (1-50)，默认 10 |

#### 3.16.6 获取日志详情

```http
GET /api/v1/audit/{log_id}
```

#### 3.16.7 清理旧日志 (管理员)

```http
POST /api/v1/audit/cleanup
```

**需要认证**: 是 (admin)

**查询参数**:
| 参数 | 类型 | 说明 |
|------|------|------|
| days | int | 保留天数 (30-365)，默认 90 |

---

## 4. Data Plane - WebSocket API

**连接地址**: `ws://{host}:8765`

### 4.1 连接认证流程

1. 建立 WebSocket 连接
2. 发送 `AuthRequest` 消息
3. 收到 `AuthResponse` 确认
4. 开始订阅/发送消息

### 4.2 消息类型枚举

所有消息使用 Protobuf 编码，封装在 `WebSocketMessage` 结构中。

#### 认证与订阅 (0x0001-0x002F)

| 消息类型 | 值 | 方向 | 说明 |
|----------|-----|------|------|
| `MSG_AUTH_REQUEST` | 1 | C→S | 认证请求 (携带 JWT) |
| `MSG_AUTH_RESPONSE` | 2 | S→C | 认证响应 |
| `MSG_SUBSCRIBE` | 16 | C→S | 订阅话题 |
| `MSG_UNSUBSCRIBE` | 17 | C→S | 取消订阅 |
| `MSG_HEARTBEAT` | 32 | C→S | 心跳 (<200ms 间隔) |
| `MSG_HEARTBEAT_ACK` | 33 | S→C | 心跳响应 |

#### 控制意图 (0x0100-0x01FF)

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
| `MSG_ARM_MOVE` | 268 | C→S | 机械臂 MoveJ/MoveL |
| `MSG_ARM_JOG` | 269 | C→S | 机械臂点动 |

#### 状态推送 (0x0200-0x02FF)

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
| `MSG_BASIC_STATE` | 520 | S→C | 基础状态（状态栏用） |

#### 系统通知 (0x0300-0x04FF)

| 消息类型 | 值 | 方向 | 说明 |
|----------|-----|------|------|
| `MSG_ERROR` | 768 | S→C | 错误通知 |
| `MSG_MODE_CHANGED` | 1024 | S→C | 模式变更通知 |
| `MSG_CONTROL_CHANGED` | 1025 | S→C | 控制权变更通知 |
| `MSG_EMERGENCY_STOP` | 1026 | 双向 | 紧急停止 |

### 4.3 Protobuf 消息定义

详见 `shared/proto/` 目录：
- `common.proto` - 通用类型 (Timestamp, Vector3, Pose, Header 等)
- `control.proto` - 控制消息 (VRControlIntent, ChassisVelocity, JointCommand 等)
- `state.proto` - 状态消息 (RobotState, JointState, ChassisState 等)
- `messages.proto` - 消息封装 (WebSocketMessage, MessageType)

### 4.4 心跳机制

- 客户端必须每 **200ms** 发送一次 `MSG_HEARTBEAT`
- 超时未收到心跳将触发 **Watchdog 急停**
- 服务端回复 `MSG_HEARTBEAT_ACK`

### 4.5 订阅话题

```protobuf
message SubscribeRequest {
    repeated string topics = 2;  // 要订阅的话题
    int32 max_rate_hz = 3;       // 最大推送频率
}
```

可订阅话题:
- `robot_state` - 机器人综合状态 (30Hz)
- `joint_state` - 关节状态 (100Hz)
- `arm_state` - 机械臂状态 (30Hz)
- `chassis_state` - 底盘状态 (30Hz)
- `gripper_state` - 夹爪状态 (30Hz)
- `vr_system_state` - VR 系统状态 (90Hz)
- `task_state` - 任务状态 (事件触发)
- `actuator_state` - 执行器状态 (30Hz)

---

## 5. Media Plane - WebRTC API

**信令地址**: `ws://{host}:8888`

### 5.1 信令消息格式 (JSON)

#### 请求视频流

```json
{
  "type": "request_stream",
  "camera_id": "head_rgb",
  "session_id": "abc123"
}
```

#### SDP Offer/Answer

```json
{
  "type": "offer",
  "sdp": "v=0\r\no=- ...",
  "camera_id": "head_rgb"
}
```

```json
{
  "type": "answer",
  "sdp": "v=0\r\no=- ...",
  "camera_id": "head_rgb"
}
```

#### ICE Candidate

```json
{
  "type": "ice_candidate",
  "candidate": "candidate:...",
  "sdpMid": "0",
  "sdpMLineIndex": 0
}
```

### 5.2 WebRTC 连接流程

1. 通过信令 WebSocket 连接
2. 发送认证 (携带 JWT)
3. 请求视频流 (`request_stream`)
4. 交换 SDP (offer/answer)
5. 交换 ICE candidates
6. 建立 P2P 连接，接收视频流

---

## 6. 错误码参考

### 6.1 通用错误 (1000-1008)

| 错误码 | 说明 |
|--------|------|
| 1000 | 未知错误 |
| 1001 | 验证错误 |
| 1002 | 资源不存在 |
| 1003 | 内部服务器错误 |
| 1004 | 权限不足 |
| 1005 | 资源不存在（别名） |
| 1006 | 参数无效 |
| 1007 | 操作失败 |
| 1008 | 禁止访问 |

### 6.2 认证错误 (2001-2005)

| 错误码 | 说明 |
|--------|------|
| 2001 | 无效的凭据 |
| 2002 | Token 已过期 |
| 2003 | Token 无效 |
| 2004 | 用户已被禁用 |
| 2005 | 权限不足 |

### 6.3 控制权错误 (3001-3003)

| 错误码 | 说明 |
|--------|------|
| 3001 | 控制权已被占用 |
| 3002 | 未持有控制权 |
| 3003 | 控制权超时 |

### 6.4 模式错误 (4001-4002)

| 错误码 | 说明 |
|--------|------|
| 4001 | 无效的模式转换 |
| 4002 | 当前模式不允许该操作 |

### 6.5 任务错误 (5001-5004)

| 错误码 | 说明 |
|--------|------|
| 5001 | 任务不存在 |
| 5002 | 任务已在运行 |
| 5003 | 任务未在运行 |
| 5004 | 任务状态无效 |

### 6.6 系统错误 (6001-6005)

| 错误码 | 说明 |
|--------|------|
| 6001 | ROS2 未连接 |
| 6002 | 机器人处于错误状态 |
| 6003 | 急停已触发 |
| 6004 | ROS2 服务调用失败 |
| 6005 | ROS2 调用错误 |

---

## 附录

### A. 频率要求汇总

| 操作类型 | 协议 | 频率 | 延迟要求 |
|----------|------|------|----------|
| 认证/配置 | HTTP | <5Hz | 100-500ms |
| 底盘速度 | WebSocket | 50Hz | <20ms |
| 机械臂控制 | WebSocket | 50-100Hz | <10ms |
| 心跳 | WebSocket | >5Hz | <200ms |
| 状态推送 | WebSocket | 30-100Hz | <10ms |
| 视频流 | WebRTC | 30fps | <100ms |

### B. 禁止的设计模式

- ❌ 在 FastAPI 中提供速度命令接口
- ❌ 在 FastAPI 中提供高频急停接口
- ❌ 在 FastAPI 中提供关节控制接口
- ❌ 通过 HTTP 轮询获取实时状态
- ❌ 通过 HTTP 传输视频帧
