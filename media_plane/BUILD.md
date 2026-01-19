# Media Plane 编译与运行指南

## 项目简介

Media Plane 是 QYH Jushen 机器人系统的视频流服务，提供：
- WebRTC 实时视频流传输
- ROS2 摄像头话题订阅
- GStreamer 视频编码管线
- WebSocket 信令服务器
- JWT 身份验证

**技术栈**: C++17 | GStreamer 1.20+ | Boost.Beast | ROS2 Humble | WebRTC

---

## 系统要求

### 必需依赖

| 依赖项 | 最低版本 | 说明 |
|--------|---------|------|
| CMake | 3.16+ | 构建工具 |
| GCC/Clang | C++17 支持 | 编译器 |
| GStreamer | 1.20+ | 视频处理框架 |
| Boost | 任意版本 | WebSocket、线程 |
| yaml-cpp | 0.6+ | 配置解析 |
| nlohmann_json | 3.9+ | JSON 处理 |
| OpenSSL | 1.1+ | JWT 验证 |
| ROS2 Humble | - | 图像源（可选） |

### GStreamer 插件

```bash
sudo apt install \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools
```

---

## 快速开始

### 1. 环境准备

```bash
# Source ROS2（如需 ROS2 图像源）
source /opt/ros/humble/setup.bash

# 验证 GStreamer
gst-inspect-1.0 webrtcbin  # 应该显示 webrtcbin 插件信息
```

### 2. 编译步骤

```bash
cd /home/yczn/qyh-robot-system/qyh_jushen_backend/media_plane

# 创建 build 目录
mkdir -p build && cd build

# 配置
cmake ..

# 编译
make -j$(nproc)

# 检查生成的可执行文件
ls -lh media_plane_server
```

### 3. 运行服务

```bash
# 从 build 目录运行
./media_plane_server

# 或指定配置文件
./media_plane_server --config ../config/config.yaml
```

**默认监听**: `ws://0.0.0.0:8765`（WebSocket 信令）

---

## 常见问题

### 问题 1: `gstreamer-webrtc-1.0: not found`

**错误信息**:
```
Could not find gstreamer-webrtc-1.0
```

**解决方案**:
```bash
sudo apt install libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-bad
pkg-config --modversion gstreamer-webrtc-1.0  # 验证安装
```

---

### 问题 2: ROS2 not found (可选)

**警告信息**:
```
ROS2 not found, ROS2 image source will be disabled
```

**说明**: 这不是错误。如果不需要订阅 ROS2 摄像头话题，可以忽略此警告。

**如需启用 ROS2**:
```bash
source /opt/ros/humble/setup.bash
source ~/qyh-robot-system/qyh_jushen_ws/install/setup.bash
cmake ..
```

---

### 问题 3: 编译错误 - `create_offer()` 返回类型

**错误信息**:
```
error: could not convert 'webrtc_peer_->create_offer()' from 'void' to 'bool'
```

**原因**: 代码使用了过时的 API 调用方式

**解决方案**: 已在代码中修复 - `create_offer()` 和 `set_remote_description()` 现在正确地作为 `void` 函数调用

---

### 问题 4: `make_strand` 未定义

**错误信息**:
```
error: 'make_strand' is not a member of 'qyh::mediaplane::net'
```

**解决方案**: 已修复 - 使用 Boost 1.74 兼容的 socket 创建方式

---

### 问题 5: WebRTC 库不稳定警告

**警告信息**:
```
warning: The WebRTC library from gst-plugins-bad is unstable API
```

**说明**: 这是 GStreamer WebRTC 插件的正常警告，不影响功能

**消除警告（可选）**:
```cmake
# 在 CMakeLists.txt 中添加
add_definitions(-DGST_USE_UNSTABLE_API)
```

---

## 配置文件

### config/config.yaml 示例

```yaml
server:
  host: "0.0.0.0"
  port: 8765
  max_connections: 50

webrtc:
  stun_server: "stun:stun.l.google.com:19302"
  ice_servers:
    - "stun:stun1.l.google.com:19302"
    - "stun:stun2.l.google.com:19302"

video:
  # 使用 ROS2 摄像头话题
  source: "ros2"
  ros2_topic: "/camera/image_raw"
  
  # 或使用测试源
  # source: "test"
  
  codec: "VP8"
  bitrate: 2000000  # 2 Mbps
  framerate: 30

auth:
  jwt_secret: "your-secret-key"
  enabled: true
```

---

## 架构说明

```
media_plane_server
├── Signaling Server (WebSocket)
│   ├── 监听 ws://0.0.0.0:8765
│   ├── 处理 WebRTC 信令（Offer/Answer/ICE）
│   └── JWT 身份验证
│
├── Pipeline Manager (GStreamer)
│   ├── 视频编码管线
│   ├── WebRTC 数据通道
│   └── RTP 打包
│
├── ROS2 Image Source (可选)
│   ├── 订阅: /camera/image_raw
│   └── 转换: sensor_msgs/Image → GstBuffer
│
└── WebRTC Peer
    ├── SDP 协商
    ├── ICE 候选交换
    └── DTLS/SRTP 加密
```

---

## WebRTC 工作流程

### 1. 客户端连接
```
Client → WebSocket → SignalingServer
```

### 2. 创建会话
```json
{
  "type": "start_stream",
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

### 3. SDP 交换
```
Server → Offer (SDP)
Client → Answer (SDP)
```

### 4. ICE 协商
```
Client ⇄ ICE Candidates ⇄ Server
```

### 5. 视频传输
```
ROS2 Camera → GStreamer → WebRTC → Client
```

---

## 开发调试

### 启用 GStreamer 调试

```bash
export GST_DEBUG=3  # 1-5, 5 最详细
export GST_DEBUG_FILE=/tmp/gst-debug.log

./media_plane_server
```

### 测试 WebRTC 连接

```javascript
// 浏览器端测试
const ws = new WebSocket('ws://localhost:8765');
const pc = new RTCPeerConnection({
  iceServers: [{urls: 'stun:stun.l.google.com:19302'}]
});

ws.onmessage = async (event) => {
  const msg = JSON.parse(event.data);
  if (msg.type === 'offer') {
    await pc.setRemoteDescription(new RTCSessionDescription(msg));
    const answer = await pc.createAnswer();
    await pc.setLocalDescription(answer);
    ws.send(JSON.stringify({type: 'answer', sdp: answer.sdp}));
  }
};

pc.ontrack = (event) => {
  document.getElementById('video').srcObject = event.streams[0];
};
```

### 查看 WebRTC 统计

```bash
# 在 GStreamer 管线中启用统计
GST_DEBUG=webrtcbin:5 ./media_plane_server
```

---

## 性能调优

### 1. 视频编码参数

```yaml
video:
  codec: "VP8"      # 或 "H264"
  bitrate: 2000000  # 增加以提高质量
  framerate: 30     # 降低以减少带宽
  width: 1280
  height: 720
```

### 2. 网络适应

```yaml
webrtc:
  adaptive_bitrate: true
  min_bitrate: 500000
  max_bitrate: 5000000
```

### 3. 延迟优化

```yaml
video:
  low_latency: true
  buffer_size: 0      # 禁用缓冲
  keyframe_interval: 30  # 增加关键帧频率
```

---

## 故障排查清单

- [ ] GStreamer 1.20+ 已安装
- [ ] webrtcbin 插件可用（`gst-inspect-1.0 webrtcbin`）
- [ ] ROS2 环境已 source（如需 ROS2 图像源）
- [ ] 端口 8765 未被占用
- [ ] 防火墙允许 WebRTC 相关端口（UDP 49152-65535）
- [ ] STUN 服务器可访问
- [ ] 客户端支持 VP8/H264 解码

---

## 已知限制

1. **单流限制**: 当前实现每个服务器实例只支持一个视频流
2. **编解码器**: 主要测试 VP8，H264 支持可能需要额外配置
3. **平台**: 在 ARM64 (Jetson) 上测试，x86_64 应该也可用
4. **带宽**: 未实现自适应比特率控制

---

## 维护者

QYH Jushen Backend Team  
最后更新: 2026-01-19
