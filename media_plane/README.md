# QYH Robot Media Plane

> **媒体平面** - 基于 C++ / GStreamer / WebRTC 的高性能视频流服务器
>
> 负责机器人多摄像头视频流的硬件编码和低延迟 WebRTC 传输

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
│   - 模式切换              - Watchdog          - 硬件编码     │
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
| **多相机视频流** | 头部、左手、右手相机同时推流 |
| **硬件编码** | Jetson NVENC H.264/H.265 编码 |
| **WebRTC 传输** | 低延迟 P2P 视频传输 (<100ms) |
| **信令服务** | WebSocket 信令交换 SDP/ICE |
| **多客户端** | 支持多个客户端同时观看 |

## 技术栈

- **语言**: C++17
- **媒体框架**: GStreamer 1.20+
- **WebRTC**: gstreamer-webrtc
- **信令**: Boost.Beast WebSocket
- **配置**: yaml-cpp
- **JSON**: nlohmann/json

## 目录结构

```
media_plane/
├── CMakeLists.txt              # 构建配置
├── config/
│   └── config.yaml             # 运行时配置
├── include/media_plane/        # 头文件
│   ├── config.hpp              # 配置结构
│   ├── media_server.hpp        # 媒体服务器主类
│   ├── pipeline_manager.hpp    # GStreamer 管道管理
│   ├── signaling_server.hpp    # WebRTC 信令服务器
│   └── webrtc_peer.hpp         # WebRTC Peer 管理
└── src/                        # 源文件
    ├── main.cpp
    ├── config.cpp
    ├── media_server.cpp
    ├── pipeline_manager.cpp
    ├── signaling_server.cpp
    └── webrtc_peer.cpp
```

## 构建

### 依赖

```bash
# Ubuntu / Debian
sudo apt install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-nice \
    libboost-all-dev \
    libyaml-cpp-dev \
    nlohmann-json3-dev

# Jetson 额外
# gstreamer1.0-plugins-nvarguscamera (预装)
```

### 编译

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 运行

```bash
# 使用默认配置
./media_plane_server

# 指定配置文件
./media_plane_server /etc/qyh-robot/media_plane/config.yaml
```

## 配置文件

```yaml
server:
  host: "0.0.0.0"
  signaling_port: 8888
  max_connections: 10
  max_message_bytes: 1048576
  auth_timeout_sec: 10
  jwt_secret: "${JWT_SECRET}"
  require_auth: true
  auth_audience: ""
  auth_issuer: ""
  auth_scope: ""

video:
  default_source: "head_camera"
  sources:
    - name: "head_camera"
      type: "ros2"
      topic: "/head_camera/color/image_raw"
      enabled: true
    - name: "left_camera"
      type: "ros2"
      topic: "/left_camera/color/image_raw"
      enabled: true
    - name: "right_camera"
      type: "ros2"
      topic: "/right_camera/color/image_raw"
      enabled: true
    - name: "test_pattern"
      type: "test"
      enabled: true

  encoding:
    codec: "h264"
    hardware_encoder: true
    width: 1280
    height: 720
    framerate: 30
    bitrate: 4000         # kbps
    keyframe_interval: 30

webrtc:
  stun_servers:
    - "stun:stun.l.google.com:19302"
  turn_servers: []
  ice_transport_policy: "all"

jetson:
  use_nvenc: true
  nvenc_preset: "fast"

logging:
  level: "INFO"
  file: "/var/log/qyh/media_plane.log"
```

## WebRTC 信令协议

### 认证流程

当 `require_auth: true` 时，客户端使用登录获得的 JWT Token
在信令通道发送 `auth` 消息。

### 连接流程

```
Client                    Signaling Server              GStreamer Pipeline
   │                            │                              │
   │ ────── WebSocket ────────> │                              │
   │ <───── welcome ─────────── │                              │
   │                            │                              │
   │ ── request_stream ───────> │                              │
   │    (source: "head")        │ ── create_peer() ──────────> │
   │                            │ <── Offer created ────────── │
   │ <───── offer (SDP) ─────── │                              │
   │                            │                              │
   │ ────── answer (SDP) ─────> │ ── set_remote_desc() ──────> │
   │                            │                              │
   │ <── ice_candidate ──────── │ <── ICE candidate ────────── │
   │ ────── ice_candidate ────> │ ── add_ice_candidate() ───> │
   │                            │                              │
   │ <═══════════════ WebRTC Media Stream ═══════════════════> │
```

### 消息类型

| 类型 | 方向 | 说明 |
|------|------|------|
| `welcome` | S→C | 连接建立，返回 peer_id |
| `request_stream` | C→S | 请求视频流 |
| `offer` | S→C | SDP Offer |
| `answer` | C→S | SDP Answer |
| `ice_candidate` | 双向 | ICE Candidate |
| `error` | S→C | 错误信息 |

### 消息格式

```json
// welcome
{"type": "welcome", "peer_id": "abc123"}

// request_stream
{"type": "request_stream", "source": "head_camera"}

// offer / answer
{"type": "offer", "sdp": "v=0\r\n..."}
{"type": "answer", "sdp": "v=0\r\n..."}

// ice_candidate
{
  "type": "ice_candidate",
  "candidate": "candidate:...",
  "sdp_mid": "0",
  "sdp_mline_index": 0
}
```

## GStreamer Pipeline

### Jetson 硬件编码 Pipeline

```
v4l2src device=/dev/video0
    ↓
video/x-raw,width=1280,height=720,framerate=30/1
    ↓
nvvidconv (色彩空间转换)
    ↓
video/x-raw(memory:NVMM),format=NV12
    ↓
nvv4l2h264enc (Jetson NVENC)
  - bitrate=4000000
  - preset-level=1 (UltraFast)
    ↓
rtph264pay config-interval=1 pt=96
    ↓
webrtcbin (WebRTC 传输)
```

### 软件编码 Pipeline (后备)

```
v4l2src / videotestsrc
    ↓
videoconvert
    ↓
x264enc
  - bitrate=4000
  - tune=zerolatency
  - speed-preset=ultrafast
    ↓
rtph264pay
    ↓
webrtcbin
```

## 性能指标

| 指标 | 目标值 |
|------|--------|
| 端到端延迟 | < 100ms |
| 视频分辨率 | 1280x720 @ 30fps |
| 码率 | 2-4 Mbps (可调) |
| 最大并发客户端 | 10 |
| CPU 占用 | < 20% (使用硬件编码) |
| GPU 编码负载 | < 30% |

## 测试

### 测试视频源

无摄像头时可使用测试源：

```yaml
video_sources:
  - name: "test"
    device: ""
    type: "videotestsrc"
    enabled: true
```

### 前端测试页面

```html
<!-- 简单 WebRTC 测试 -->
<video id="video" autoplay playsinline></video>
<script>
const ws = new WebSocket('ws://localhost:8888');
const pc = new RTCPeerConnection({
  iceServers: [{urls: 'stun:stun.l.google.com:19302'}]
});

pc.ontrack = (e) => {
  document.getElementById('video').srcObject = e.streams[0];
};

ws.onmessage = async (e) => {
  const msg = JSON.parse(e.data);
  if (msg.type === 'offer') {
    await pc.setRemoteDescription({type: 'offer', sdp: msg.sdp});
    const answer = await pc.createAnswer();
    await pc.setLocalDescription(answer);
    ws.send(JSON.stringify({type: 'answer', sdp: answer.sdp}));
  } else if (msg.type === 'ice_candidate') {
    await pc.addIceCandidate(msg);
  }
};

pc.onicecandidate = (e) => {
  if (e.candidate) {
    ws.send(JSON.stringify({
      type: 'ice_candidate',
      candidate: e.candidate.candidate,
      sdp_mid: e.candidate.sdpMid,
      sdp_mline_index: e.candidate.sdpMLineIndex
    }));
  }
};

ws.onopen = () => {
  ws.send(JSON.stringify({type: 'request_stream', source: 'head_camera'}));
};
</script>
```

## 相关文档

- [重构.md](../重构.md) - 架构设计原则
- [重构开发计划.md](../重构开发计划.md) - 整体开发计划
- [Control Plane README](../control_plane/README.md)
- [Data Plane README](../data_plane/README.md)

---

*最后更新：2025-01-19*
