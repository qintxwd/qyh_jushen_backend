# QYH Robot Backend 架构重构计划

## 📋 文档信息

| 项目 | 内容 |
|------|------|
| 版本 | v2.0 |
| 日期 | 2026-01-31 |
| 作者 | QYH Team |
| 状态 | 规划中 |

---

## 🎯 重构目标

### 核心理念变化

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          BEFORE (当前架构)                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   Client ─────HTTP────→ Control Plane (认证/配置/任务)                   │
│           ─────WS──────→ Data Plane   (实时控制/状态 50-100Hz)           │
│           ─────WebRTC──→ Media Plane  (视频流)                           │
│                                                                          │
│   问题：三条独立连接，复杂度高，状态同步困难                                │
└─────────────────────────────────────────────────────────────────────────┘

                                    ↓ 重构 ↓

┌─────────────────────────────────────────────────────────────────────────┐
│                           AFTER (目标架构)                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   Client ─────HTTP────→ Signaling Server (认证/配置/信令)                │
│           ─────────────────────────────────────────                      │
│                    ↓ WebRTC 信令交换 ↓                                   │
│           ─────────────────────────────────────────                      │
│           ←───QUIC/DTLS───→ Robot (直连)                                │
│              • Media Tracks (视频)                                       │
│              • DataChannel 1 (控制命令)                                  │
│              • DataChannel 2 (状态推送)                                  │
│              • DataChannel 3 (事件/日志)                                 │
│                                                                          │
│   优势：统一连接，低延迟，P2P直连，NAT穿透                                 │
└─────────────────────────────────────────────────────────────────────────┘
```

### 关键收益

| 收益 | 说明 |
|------|------|
| **统一传输** | Video + Data 走同一 WebRTC 连接，简化状态管理 |
| **超低延迟** | QUIC/DTLS 直连，端到端延迟 <50ms |
| **NAT 穿透** | ICE 框架自动处理 NAT，支持任意网络环境 |
| **简化架构** | 去掉独立的 WebSocket Data Plane，降低复杂度 |
| **带宽自适应** | WebRTC 内置拥塞控制，自动适应网络状况 |

---

## 🏗️ 新架构设计

### 整体架构图

```
                    ┌─────────────────────────────────────────┐
                    │           Client Layer                   │
                    │      (Web / VR / Mobile App)             │
                    └──────────────┬──────────────────────────┘
                                   │
         ┌─────────────────────────┼─────────────────────────┐
         │                         │                         │
         ▼                         ▼                         │
┌─────────────────┐    ┌────────────────────────┐            │
│  /login API     │    │   /signaling API       │            │
│  (HTTP/REST)    │    │   (WebSocket)          │            │
│                 │    │                        │            │
│  • 用户认证     │    │  • SDP Offer/Answer    │            │
│  • Token 签发   │    │  • ICE Candidate       │            │
│  • 机器人列表   │    │  • 会话路由            │            │
└────────┬────────┘    └───────────┬────────────┘            │
         │                         │                         │
         │      ┌──────────────────┘                         │
         │      │                                            │
         ▼      ▼                                            │
┌─────────────────────────────────────────────────────────┐  │
│              Signaling Server (FastAPI)                  │  │
│                                                          │  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │  │
│  │ Auth Module │  │Session Mgmt │  │  Robot Registry │  │  │
│  └─────────────┘  └─────────────┘  └─────────────────┘  │  │
│                                                          │  │
│  ┌─────────────────────────────────────────────────────┐ │  │
│  │           Signaling Relay (SDP/ICE 转发)            │ │  │
│  └─────────────────────────────────────────────────────┘ │  │
└──────────────────────────┬──────────────────────────────┘  │
                           │                                  │
        ┌──────────────────┴──────────────────┐              │
        │           信令交换完成后              │              │
        │                                      │              │
        ▼                                      ▼              │
┌──────────────────┐                ┌──────────────────┐     │
│     Client       │◄══════════════►│     Robot        │     │
│  RTCPeerConn     │   QUIC/DTLS    │  RTCPeerConn     │     │
│                  │   直连通道     │                  │     │
│ • MediaTrack     │                │ • MediaTrack     │     │
│ • DataChannel×3  │                │ • DataChannel×3  │     │
└──────────────────┘                └──────────────────┘     │
                                                             │
                                    │                        │
                                    ▼                        │
                    ┌───────────────────────────────────────┐│
                    │          ROS2 Runtime Layer          ││
                    │   Drivers / Control Loop / Safety    ││
                    └───────────────────────────────────────┘│
```

### 组件职责重新划分

#### 1. Signaling Server (原 Control Plane 演进)

| 模块 | 职责 | 接口 |
|------|------|------|
| **Auth** | 用户认证、Token 签发 | `POST /login`, `POST /logout`, `POST /refresh` |
| **Robot Registry** | 机器人注册、在线状态、WebRTC 配置 | `GET /robots`, `GET /robots/{id}/webrtc-config` |
| **Session Manager** | 控制会话管理、权限控制 | `POST /session/acquire`, `DELETE /session/release` |
| **Signaling Relay** | SDP/ICE 信令转发 | `WS /signaling/{robot_id}` |
| **Config API** | 配置管理（保留低频 HTTP） | `GET/PUT /config/*` |
| **Task API** | 任务 CRUD（保留低频 HTTP） | `CRUD /tasks/*` |
| **Preset API** | 预设 CRUD（保留低频 HTTP） | `CRUD /presets/*` |

#### 2. Robot WebRTC Endpoint (原 Data Plane + Media Plane 合并)

| 模块 | 职责 | 传输通道 |
|------|------|----------|
| **Media Tracks** | 多路视频流（头部/左手/右手相机） | WebRTC Video Track |
| **Control Channel** | 实时控制命令（关节/底盘/夹爪） | DataChannel #1 (Ordered, Reliable) |
| **State Channel** | 状态推送（关节/IMU/TF/电池） | DataChannel #2 (Unordered, Unreliable) |
| **Event Channel** | 事件/日志/告警 | DataChannel #3 (Ordered, Reliable) |

#### 3. DataChannel 设计

```
┌────────────────────────────────────────────────────────────────┐
│                    DataChannel 配置                             │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  Channel #1: "control" (控制通道)                               │
│  ├─ ordered: true                                              │
│  ├─ maxRetransmits: 0 (最新命令优先)                           │
│  ├─ protocol: "protobuf"                                       │
│  └─ 用途: 底盘速度、关节命令、夹爪、急停、导航                    │
│                                                                │
│  Channel #2: "state" (状态通道)                                 │
│  ├─ ordered: false                                             │
│  ├─ maxRetransmits: 0 (允许丢包，要最新)                        │
│  ├─ protocol: "protobuf"                                       │
│  └─ 用途: 关节状态、IMU、TF、电池、底盘 odom                     │
│                                                                │
│  Channel #3: "event" (事件通道)                                 │
│  ├─ ordered: true                                              │
│  ├─ maxRetransmits: 3 (可靠传输)                               │
│  ├─ protocol: "protobuf"                                       │
│  └─ 用途: 模式变更、任务完成、错误、日志                         │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

---

## 📁 目录结构变更

### 现有结构

```
qyh_jushen_backend/
├── control_plane/      # Python/FastAPI - 保留并改造
├── data_plane/         # C++/WebSocket  - 废弃，功能迁移到 robot_endpoint
├── media_plane/        # C++/GStreamer  - 废弃，功能迁移到 robot_endpoint
└── shared/proto/       # Protobuf 定义 - 扩展
```

### 目标结构

```
qyh_jushen_backend/
├── signaling_server/           # Python/FastAPI (原 control_plane 演进)
│   ├── app/
│   │   ├── main.py
│   │   ├── config.py
│   │   ├── database.py
│   │   ├── api/
│   │   │   └── v1/
│   │   │       ├── auth.py           # 认证
│   │   │       ├── robots.py         # 机器人注册/列表
│   │   │       ├── session.py        # 控制会话管理
│   │   │       ├── signaling.py      # WebSocket 信令 (新)
│   │   │       ├── config.py         # 配置 API
│   │   │       ├── tasks.py          # 任务 CRUD
│   │   │       └── presets.py        # 预设 CRUD
│   │   ├── core/
│   │   │   ├── security.py           # JWT
│   │   │   ├── session_manager.py    # 会话管理 (新)
│   │   │   └── robot_registry.py     # 机器人注册中心 (新)
│   │   └── services/
│   │       └── signaling_relay.py    # 信令转发服务 (新)
│   └── requirements.txt
│
├── robot_endpoint/             # C++/GStreamer/WebRTC (合并 data+media)
│   ├── CMakeLists.txt
│   ├── config/
│   │   └── config.yaml
│   ├── include/robot_endpoint/
│   │   ├── config.hpp
│   │   ├── webrtc_server.hpp         # WebRTC 主服务
│   │   ├── signaling_client.hpp      # 连接 Signaling Server
│   │   ├── peer_connection.hpp       # RTCPeerConnection 封装
│   │   ├── media_track_manager.hpp   # 视频轨道管理
│   │   ├── data_channel_manager.hpp  # DataChannel 管理 (新)
│   │   ├── control_handler.hpp       # 控制命令处理
│   │   ├── state_publisher.hpp       # 状态推送
│   │   ├── ros2_bridge.hpp           # ROS2 桥接
│   │   └── watchdog.hpp              # 安全看门狗
│   └── src/
│       ├── main.cpp
│       ├── webrtc_server.cpp
│       ├── signaling_client.cpp
│       ├── peer_connection.cpp
│       ├── media_track_manager.cpp
│       ├── data_channel_manager.cpp
│       ├── control_handler.cpp
│       ├── state_publisher.cpp
│       ├── ros2_bridge.cpp
│       └── watchdog.cpp
│
├── shared/
│   └── proto/
│       ├── common.proto              # 保留
│       ├── control.proto             # 保留 - 需优化为 ControlMessage 顶层封装
│       ├── state.proto               # 保留 - 需优化为 StateMessage 顶层封装
│       ├── signaling.proto           # 新增：信令消息
│       └── event.proto               # 新增：事件消息
│
└── deploy/
    ├── systemd/
    │   ├── qyh-signaling-server.service
    │   └── qyh-robot-endpoint.service
    └── install.sh
```

---

## 🔄 重构阶段计划

### Phase 1: 信令服务器改造 (预计 2 周)

#### 1.1 创建信令 WebSocket 端点

**目标**: 在 Signaling Server 中增加 `/signaling/{robot_id}` WebSocket 端点

**任务清单**:

- [ ] 创建 `app/api/v1/signaling.py`
  - WebSocket 连接处理
  - SDP Offer/Answer 转发
  - ICE Candidate 转发
- [ ] 创建 `app/core/robot_registry.py`
  - 机器人在线状态管理
  - 机器人 WebRTC 能力注册
- [ ] 创建 `app/core/session_manager.py`
  - Client ↔ Robot 会话配对
  - 控制权管理
- [ ] 扩展 `shared/proto/signaling.proto`

**新增代码示例**:

```python
# app/api/v1/signaling.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Depends
from app.core.security import verify_ws_token
from app.core.robot_registry import RobotRegistry
from app.services.signaling_relay import SignalingRelay

router = APIRouter()

@router.websocket("/signaling/{robot_id}")
async def signaling_endpoint(
    websocket: WebSocket,
    robot_id: str,
    token: str = Query(...),
):
    """
    WebRTC 信令 WebSocket 端点
    
    消息类型:
    - offer: SDP Offer
    - answer: SDP Answer  
    - ice_candidate: ICE Candidate
    - error: 错误信息
    """
    # 验证 Token
    user = await verify_ws_token(token)
    if not user:
        await websocket.close(code=4001, reason="Unauthorized")
        return
    
    # 检查机器人是否在线
    robot_registry = RobotRegistry()
    if not robot_registry.is_online(robot_id):
        await websocket.close(code=4004, reason="Robot offline")
        return
    
    await websocket.accept()
    
    # 创建信令会话
    relay = SignalingRelay()
    session_id = await relay.create_session(user.id, robot_id, websocket)
    
    try:
        while True:
            data = await websocket.receive_json()
            await relay.handle_message(session_id, data)
    except WebSocketDisconnect:
        await relay.close_session(session_id)
```

```protobuf
// shared/proto/signaling.proto
syntax = "proto3";
package qyh.signaling;

message SignalingMessage {
    string session_id = 1;
    oneof payload {
        SDPMessage sdp = 2;
        ICECandidate ice = 3;
        ErrorMessage error = 4;
    }
}

message SDPMessage {
    string type = 1;  // "offer" or "answer"
    string sdp = 2;
}

message ICECandidate {
    string candidate = 1;
    string sdp_mid = 2;
    int32 sdp_mline_index = 3;
}

message ErrorMessage {
    int32 code = 1;
    string message = 2;
}
```

#### 1.2 机器人注册 API

**任务清单**:

- [ ] 创建 `GET /api/v1/robots` - 获取机器人列表
- [ ] 创建 `GET /api/v1/robots/{id}` - 获取机器人详情
- [ ] 创建 `GET /api/v1/robots/{id}/webrtc-config` - 获取 WebRTC 配置
- [ ] 创建 `POST /api/v1/robots/{id}/register` - 机器人自注册 (内部)

**数据模型**:

```python
# app/schemas/robot.py (扩展)
class RobotWebRTCConfig(BaseModel):
    """机器人 WebRTC 配置"""
    signaling_url: str           # WebSocket 信令地址
    ice_servers: List[ICEServer] # STUN/TURN 服务器
    video_sources: List[str]     # 可用视频源
    data_channels: List[DataChannelConfig]  # DataChannel 配置

class DataChannelConfig(BaseModel):
    label: str
    ordered: bool
    max_retransmits: Optional[int]
    protocol: str
```

#### 1.3 STUN/TURN 服务 (关键基础设施)

**任务清单**:

- [ ] 部署/复用 Coturn 服务 (UDP/TCP 3478, 5349)
- [ ] 创建 `app/services/turn_service.py` 生成短期凭据
- [ ] 在 `GET /robots/{id}/webrtc-config` 中返回动态 TURN 凭据

---

### Phase 2: Robot Endpoint 开发 (预计 3 周)

#### 2.1 项目初始化

**任务清单**:

- [ ] 创建 `robot_endpoint/` 目录结构
- [ ] 配置 CMakeLists.txt（集成 GStreamer + libdatachannel/libwebrtc）
- [ ] 实现配置管理模块

**依赖选择**:

| 组件 | 选项 | 推荐 | 理由 |
|------|------|------|------|
| WebRTC | libwebrtc / libdatachannel | libdatachannel | 轻量、易集成、支持 DataChannel |
| Media | GStreamer + webrtcbin | GStreamer | 成熟、硬件编码支持好 |
| Signaling | Boost.Beast WebSocket | Boost.Beast | 已在用，保持一致 |

#### 2.2 Signaling Client

**任务清单**:

- [ ] 实现 `signaling_client.hpp/cpp`
  - 连接 Signaling Server
  - 处理 SDP/ICE 消息
  - 断线重连

```cpp
// include/robot_endpoint/signaling_client.hpp
class SignalingClient : public std::enable_shared_from_this<SignalingClient> {
public:
    using OnOfferCallback = std::function<void(const std::string& sdp, const std::string& session_id)>;
    using OnICECallback = std::function<void(const std::string& candidate, 
                                              const std::string& sdp_mid, 
                                              int sdp_mline_index)>;
    
    SignalingClient(net::io_context& io, const Config& config);
    
    void connect();
    void disconnect();
    
    void send_answer(const std::string& session_id, const std::string& sdp);
    void send_ice_candidate(const std::string& session_id, 
                            const std::string& candidate,
                            const std::string& sdp_mid,
                            int sdp_mline_index);
    
    void set_on_offer(OnOfferCallback cb);
    void set_on_ice(OnICECallback cb);
    
private:
    void on_message(const std::string& message);
    void do_reconnect();
};
```

#### 2.3 DataChannel Manager 与协议优化

**Proto 拆分设计**:

不再使用统一的 `WebSocketMessage`，而是为每个 Channel 定义专用 Envelope：

```protobuf
// shared/proto/control.proto
message ControlChannelMessage {
    uint64 sequence_id = 1;
    oneof payload {
        ChassisVelocity chassis_vel = 2;
        JointCommand joint_cmd = 3;
        GripperCommand gripper = 4;
        // ...只包含控制指令
    }
}

// shared/proto/state.proto
message StateChannelMessage {
    uint64 timestamp = 1;
    oneof payload {
        RobotState full_state = 2;
        ChassisState chassis = 3;
        // ...只包含状态推送
    }
}
```

**任务清单**:

- [ ] 实现 `data_channel_manager.hpp/cpp`
  - 管理三个 DataChannel
  - Protobuf 序列化/反序列化
  - 消息路由

```cpp
// include/robot_endpoint/data_channel_manager.hpp
class DataChannelManager {
public:
    enum class Channel {
        CONTROL = 0,  // 控制命令
        STATE = 1,    // 状态推送  
        EVENT = 2     // 事件通知
    };
    
    using MessageCallback = std::function<void(Channel, const std::vector<uint8_t>&)>;
    
    DataChannelManager(rtc::PeerConnection* pc);
    
    void setup_channels();
    
    // 发送消息
    void send_control(const google::protobuf::Message& msg);
    void send_state(const google::protobuf::Message& msg);
    void send_event(const google::protobuf::Message& msg);
    
    // 接收回调
    void set_on_message(MessageCallback cb);
    
private:
    std::shared_ptr<rtc::DataChannel> control_channel_;
    std::shared_ptr<rtc::DataChannel> state_channel_;
    std::shared_ptr<rtc::DataChannel> event_channel_;
};
```

#### 2.4 集成原有功能

**从 data_plane 迁移**:

- [ ] `message_handler.cpp` → `control_handler.cpp`
- [ ] `state_cache.cpp` → `state_publisher.cpp`
- [ ] `ros2_bridge.cpp` → 保持
- [ ] `watchdog.cpp` → 保持
- [ ] `auth.cpp` → 简化（Token 验证移到 Signaling Server）

**从 media_plane 迁移**:

- [ ] `pipeline_manager.cpp` → `media_track_manager.cpp`
- [ ] `webrtc_peer.cpp` → `peer_connection.cpp`

### Phase 2.5: 开发辅助工具 (Mock Robot)

**这对于前端开发至关重要，让他们无需等待 C++ 端完成即可开工。**

**任务清单**:
- [ ] 创建 `tools/mock_robot/main.py` (使用 `aiortc` 库)
- [ ] 实现模拟信令交互
- [ ] 实现模拟视频流 (读取本地 MP4 循环播放)
- [ ] 实现模拟状态推送 (发送正弦波运动数据)

---

### Phase 3: 前端适配 (预计 1-2 周)

#### 3.1 WebRTC 客户端重构

**任务清单**:

- [ ] 创建统一的 `WebRTCConnection` 类
- [ ] 实现 DataChannel 消息处理
- [ ] 移除原有 WebSocket 连接逻辑

**代码示例**:

```typescript
// src/services/WebRTCConnection.ts
export class WebRTCConnection {
    private pc: RTCPeerConnection | null = null;
    private controlChannel: RTCDataChannel | null = null;
    private stateChannel: RTCDataChannel | null = null;
    private eventChannel: RTCDataChannel | null = null;
    private signalingWs: WebSocket | null = null;
    
    async connect(robotId: string, token: string) {
        // 1. 连接信令服务器
        this.signalingWs = new WebSocket(
            `wss://${SIGNALING_HOST}/api/v1/signaling/${robotId}?token=${token}`
        );
        
        // 2. 创建 PeerConnection
        this.pc = new RTCPeerConnection({
            iceServers: await this.getICEServers()
        });
        
        // 3. 等待 DataChannel
        this.pc.ondatachannel = (event) => {
            this.setupDataChannel(event.channel);
        };
        
        // 4. 等待来自 Robot 的 Offer
        this.signalingWs.onmessage = async (event) => {
            const msg = JSON.parse(event.data);
            await this.handleSignalingMessage(msg);
        };
    }
    
    private setupDataChannel(channel: RTCDataChannel) {
        switch (channel.label) {
            case 'control':
                this.controlChannel = channel;
                break;
            case 'state':
                this.stateChannel = channel;
                channel.onmessage = (e) => this.onStateMessage(e.data);
                break;
            case 'event':
                this.eventChannel = channel;
                channel.onmessage = (e) => this.onEventMessage(e.data);
                break;
        }
    }
    
    // 发送控制命令
    sendChassisVelocity(vx: number, vy: number, omega: number) {
        const msg = ChassisVelocity.create({ vx, vy, omega });
        this.controlChannel?.send(ChassisVelocity.encode(msg).finish());
    }
}
```

#### 3.2 视频播放器适配

- [ ] 统一使用 `RTCPeerConnection` 的 `ontrack` 事件
- [ ] 移除原有的独立 WebRTC 信令逻辑

---

### Phase 4: 测试与迁移 (预计 1 周)

#### 4.1 测试计划

| 测试类型 | 内容 | 工具 |
|----------|------|------|
| 单元测试 | DataChannel 消息编解码 | GoogleTest |
| 集成测试 | 信令流程、连接建立 | pytest + Playwright |
| 性能测试 | 延迟、吞吐量 | 自定义脚本 |
| NAT 穿透测试 | 不同网络环境 | 多设备测试 |

#### 4.2 迁移策略与构建更新

**构建脚本更新**:

必须彻底重写 `build_all.sh`，使其适配新的组件结构：

```bash
# build_all.sh (示意)
build_signaling_server() {
    # 纯 Python 项目，主要是整理 venv
    cd signaling_server
    # ...
}

build_robot_endpoint() {
    cd robot_endpoint && mkdir -p build
    cmake .. && make -j4
}
```

**部署架构调整**:
- `qyh-signaling.service` (Python/FastAPI)
- `qyh-robot.service` (C++ Binary)

**分阶段迁移**:

```
Week 1: 并行运行
├─ 新架构 (WebRTC All-in-one)
│   └─ 端口 8888
└─ 旧架构 (HTTP + WS + WebRTC)
    ├─ 端口 8000 (HTTP)
    ├─ 端口 8765 (WS)
    └─ 端口 8888 (WebRTC 视频)

Week 2: 灰度切换
├─ 10% 流量 → 新架构
└─ 90% 流量 → 旧架构

Week 3: 全量切换
├─ 100% 流量 → 新架构
└─ 旧架构关闭
```

---

## 🔐 安全设计

### Token 流转

```
1. Client → Signaling Server: POST /login
   └─ Response: { access_token, refresh_token }

2. Client → Signaling Server: WS /signaling/{robot_id}?token=xxx
   └─ Server 验证 Token，建立信令会话

3. Signaling Server → Robot: 转发 SDP (包含 session_id)
   └─ Robot 通过 session_id 关联客户端

4. Client ↔ Robot: WebRTC 连接建立
   └─ DataChannel 消息可选包含 session_id 做额外验证
```

### Watchdog 安全机制

```
Robot Endpoint 内置 Watchdog:

1. 心跳检测
   └─ Client 每 100ms 发送心跳 (DataChannel #1)
   └─ Robot 500ms 未收到心跳 → 触发安全停止

2. 连接状态监控
   └─ DataChannel 关闭 → 立即安全停止
   └─ ICE 连接断开 → 立即安全停止

3. 命令时效性
   └─ 每条控制命令带时间戳
   └─ 超过 200ms 的命令丢弃
```

---

## 📊 性能目标

| 指标 | 当前值 | 目标值 |
|------|--------|--------|
| 控制命令延迟 | 15-30ms (WS) | <10ms (DataChannel) |
| 状态推送延迟 | 20-40ms (WS) | <15ms (DataChannel) |
| 视频端到端延迟 | 80-150ms | <100ms |
| NAT 穿透成功率 | N/A | >95% |
| 并发连接数 | 10 | 20+ |

---

## 📅 时间线总览

```
2026-02
├─ Week 1-2: Phase 1 - 信令服务器改造
│   ├─ 信令 WebSocket 端点
│   ├─ 机器人注册 API
│   └─ 会话管理

2026-02 ~ 2026-03
├─ Week 3-5: Phase 2 - Robot Endpoint 开发
│   ├─ 项目初始化
│   ├─ Signaling Client
│   ├─ DataChannel Manager
│   └─ 功能迁移

2026-03
├─ Week 6-7: Phase 3 - 前端适配
│   ├─ WebRTC 客户端重构
│   └─ 视频播放器适配

├─ Week 8: Phase 4 - 测试与迁移
│   ├─ 集成测试
│   └─ 灰度上线
```

---

## 🗑️ 废弃组件

以下组件在新架构完成后将被废弃：

| 组件 | 原路径 | 替代方案 |
|------|--------|----------|
| Data Plane | `data_plane/` | `robot_endpoint/` DataChannel |
| Media Plane | `media_plane/` | `robot_endpoint/` MediaTrack |
| WebSocket 状态推送 | `data_plane/src/session.cpp` | DataChannel #2 |
| WebSocket 控制命令 | `data_plane/src/message_handler.cpp` | DataChannel #1 |

---

## 📚 参考资料

- [WebRTC API (MDN)](https://developer.mozilla.org/en-US/docs/Web/API/WebRTC_API)
- [libdatachannel](https://github.com/paullouisageneau/libdatachannel)
- [GStreamer WebRTC](https://gstreamer.freedesktop.org/documentation/webrtc/)
- [ICE (Interactive Connectivity Establishment)](https://tools.ietf.org/html/rfc8445)
- [QUIC Protocol](https://www.rfc-editor.org/rfc/rfc9000.html)

---

## ✅ 检查清单

### Phase 1 完成标准
- [ ] Signaling WebSocket 端点可连接
- [ ] SDP Offer/Answer 可正常转发
- [ ] ICE Candidate 可正常转发
- [ ] 机器人在线状态可查询

### Phase 2 完成标准
- [ ] Robot Endpoint 可连接 Signaling Server
- [ ] 可响应 SDP Offer 并返回 Answer
- [ ] 三个 DataChannel 可正常建立
- [ ] 视频流可正常推送
- [ ] 控制命令可正常接收并执行
- [ ] 状态数据可正常推送

### Phase 3 完成标准
- [ ] 前端可通过新架构建立连接
- [ ] 视频流正常显示
- [ ] 控制命令正常工作
- [ ] 状态数据正常接收

### Phase 4 完成标准
- [ ] 所有测试通过
- [ ] 性能达标
- [ ] 灰度上线无异常
- [ ] 文档更新完成

---

*文档结束*
