# QYH Jushen Control Plane

机器人后端服务的控制平面，使用 FastAPI 构建。负责处理低频管理请求（<5Hz），如认证、配置、任务、预设等。

## 架构概述

### 三平面架构

本项目采用三平面架构设计：

| 平面 | 语言 | 端口 | 职责 |
|------|------|------|------|
| **Control Plane** | Python/FastAPI | 8000 | 低频管理 API（本模块） |
| **Data Plane** | C++/WebSocket | 8765 | 高频控制数据流（关节控制、遥操作） |
| **Media Plane** | C++/GStreamer+WebRTC | 8888 | 视频流传输 |

### 为什么分离？

- **Control Plane**: 处理认证、配置、任务编排等低频请求，Python 开发效率高
- **Data Plane**: 50Hz+ 的关节控制需要低延迟，C++ 性能更好
- **Media Plane**: 视频编解码需要 GStreamer 原生支持，WebRTC 需要 C++ 实现

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
