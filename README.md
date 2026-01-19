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

- `POST /api/v1/auth/login` - 登录
- `GET /api/v1/system/config` - 服务发现
- `POST /api/v1/control/acquire` - 获取控制权
- `POST /api/v1/mode/switch` - 切换模式
- `GET /api/v1/tasks` - 任务列表

### Data Plane (WebSocket + Protobuf)

连接: `ws://host:8765`

消息类型:
- `AUTH_REQUEST` - 认证
- `SUBSCRIBE_REQUEST` - 订阅话题
- `HEARTBEAT` - 心跳（<200ms 间隔）
- `VR_CONTROL_INTENT` - VR 控制意图
- `ROBOT_STATE` - 机器人状态推送

### Media Plane (WebSocket + JSON)

连接: `ws://host:8888`

信令消息:
- `request_stream` - 请求视频流
- `offer/answer` - SDP 交换
- `ice_candidate` - ICE 候选

## 重要约束

1. **Watchdog 超时**: 客户端必须每 200ms 发送心跳，否则触发紧急停止
2. **控制锁**: 同一时间只有一个客户端可以控制机器人
3. **模式状态机**: 模式切换需遵循状态机规则
4. **视频不走 HTTP**: 视频必须通过 WebRTC 传输

## 配置文件

- `/etc/qyh-robot/environment` - 共享环境变量
- `/etc/qyh-robot/data_plane/config.yaml` - Data Plane 配置
- `/etc/qyh-robot/media_plane/config.yaml` - Media Plane 配置
