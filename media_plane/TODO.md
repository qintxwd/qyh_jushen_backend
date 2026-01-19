# Media Plane 开发状态报告

> 基于 `重构.md` 架构要求和 `重构开发计划.md` 设计，整理 media_plane 实现状态。
>
> **Media Plane 职责**：多摄像头视频流硬件编码和 WebRTC 低延迟传输
>
> **最后更新**: 2025-01-20

---

## 📊 实现状态总览

| 模块 | 文件 | 状态 | 说明 |
|------|------|------|------|
| **配置管理** | `config.cpp` | ✅ 完成 | YAML + 环境变量 + JWT + ROS2 |
| **媒体服务器** | `media_server.cpp` | ✅ 完成 | 信令集成完成 |
| **管道管理器** | `pipeline_manager.cpp` | ✅ 完成 | 多源+统计+ROS2+错误处理 |
| **信令服务器** | `signaling_server.cpp` | ✅ 完成 | 完整协议支持 |
| **WebRTC Peer** | `webrtc_peer.cpp` | ✅ 基本完成 | 需测试验证 |
| **认证模块** | `auth.cpp` | ✅ 完成 | JWT 验证实现 |
| **ROS2 图像源** | `ros2_image_source.cpp` | ✅ 新增 | Orbbec 摄像头支持 |

**总体进度：~98%**

---

## 🎯 重要更新：Orbbec RGBD 摄像头支持

> 项目使用奥比中光 Gemini 330 系列 RGBD 摄像头，通过 ROS2 发布图像话题。
> Media Plane 新增 ROS2 图像源适配器，可直接订阅 ROS2 图像话题并注入 GStreamer 管道。

### 摄像头配置

| 摄像头 | USB 端口 | 序列号 | ROS2 话题 |
|--------|----------|--------|-----------|
| head_camera | 2-3.1 | CP0HC5300021 | /head_camera/color/image_raw |
| right_camera | 2-3.2 | CP0BB530003J | /right_camera/color/image_raw |
| left_camera | 2-3.3.3 | CP0BB53000AT | /left_camera/color/image_raw |

### ROS2 集成架构

```
[Orbbec Camera Node] ---> [ROS2 Image Topic] ---> [ROS2ImageSource]
                                                       |
                                                       v
                                                  [appsrc] ---> [videoconvert] ---> [encoder] ---> [webrtcbin]
```

### 新增文件

- `ros2_image_source.hpp` - ROS2 图像源头文件
- `ros2_image_source.cpp` - ROS2 图像源实现
  - 订阅 sensor_msgs/msg/Image
  - 转换为 GstBuffer
  - 通过 appsrc 注入管道

---

## ✅ 已完成功能

### 1. 配置系统

- [x] VideoSourceConfig - 视频源配置
- [x] EncodingConfig - 编码配置 (H.264/H.265/VP8)
- [x] WebRTCConfig - STUN/TURN 配置
- [x] JetsonConfig - NVENC 配置
- [x] ServerConfig - 服务器配置
- [x] 环境变量覆盖支持
- [x] JWT 密钥配置 (支持环境变量 JWT_SECRET)
- [x] 认证开关配置 (require_auth)
- [x] **ROS2Config** - ROS2 配置（域 ID、话题发现间隔）
- [x] **topic 字段** - 视频源 ROS2 话题配置

### 2. 信令服务器 ✅ 完成

- [x] WebSocket 信令服务器
- [x] SignalingSession 会话管理
- [x] Peer ID 生成
- [x] 完整消息处理
  - [x] auth - JWT 认证
  - [x] request_stream - 请求视频流
  - [x] answer - SDP Answer
  - [x] ice_candidate - ICE Candidate
  - [x] stop_stream - 停止流
  - [x] switch_source - 切换视频源
  - [x] get_sources - 获取可用源
  - [x] get_stats - 获取统计信息
  - [x] ping/pong - 保活
- [x] 错误处理和响应 (send_error)
- [x] 认证状态检查
- [x] 连接数统计 (connection_count)
- [x] 可用源列表 (get_available_sources)

### 3. WebRTC Peer 基础

- [x] GStreamer webrtcbin 封装
- [x] STUN 服务器配置
- [x] create_offer() / create_answer()
- [x] set_remote_description()
- [x] add_ice_candidate()
- [x] SDP/ICE 回调

### 4. Pipeline 管理 ✅ 完成

- [x] VideoSource 结构
- [x] 视频源类型支持 (v4l2/nvarguscamerasrc/videotestsrc)
- [x] **ROS2 图像源支持** (ros2 类型)
- [x] 编码器创建 (nvv4l2h264enc/x264enc)
- [x] GMainLoop 管理
- [x] 完整视频管道创建
- [x] create_peer() 创建视频源元素
- [x] 多视频源支持

### 5. ROS2 图像源 ✅ 新增

- [x] ROS2ImageSource 类
- [x] 订阅 sensor_msgs/msg/Image
- [x] 多种编码格式支持 (rgb8/bgr8/rgba8/mono8/yuv422)
- [x] GstBuffer 转换和推送
- [x] appsrc 元素创建和配置
- [x] ROS2ImageSourceFactory 工厂类
- [x] 共享 ROS2 Node
- [x] FPS 统计

### 6. 认证模块 ✅ 完成

- [x] JWT Token 验证 (JwtVerifier)
- [x] Base64 URL 解码
- [x] Payload 解析
- [x] 过期时间检查
- [x] UserInfo 结构体

### 7. 媒体服务器集成 ✅ 完成

- [x] io_context 管理
- [x] SignalingServer 集成
- [x] io_thread 异步运行
- [x] 生命周期管理

### 8. Pipeline 统计 ✅ 完成

- [x] PipelineStats 结构体
- [x] active_peers 统计
- [x] total_peers_created 统计
- [x] error_count 统计
- [x] get_stats 消息支持

---

## 🟡 待完善功能

### 优先级 P0 (必须完成)

1. **集成测试**
   - [ ] 在 Jetson Orin Nano 上编译测试
   - [ ] Orbbec 摄像头实际测试
   - [ ] WebRTC 端到端验证

### 优先级 P1 (重要)

2. **超时处理**
   - [ ] ROS2 话题超时检测
   - [ ] WebRTC 连接超时
   - [ ] 自动重连机制

3. **编码器后备**
   - [ ] NVENC 失败时自动切换软编
   - [ ] 分辨率自适应

### 优先级 P2 (增强)

4. **监控和统计**
   - [x] 连接数统计
   - [x] Peer 创建统计
   - [x] 错误计数
   - [ ] 延迟统计
   - [ ] Prometheus 指标

5. **动态配置**
   - [ ] 运行时调整码率
   - [ ] 运行时调整分辨率
   - [ ] 热重载配置

6. **高级功能**
   - [ ] TURN 服务器支持
   - [ ] H.265 支持
   - [ ] 帧时间戳同步
   - [ ] 录制支持

---

## 📝 下一步工作

### 1. 测试验证

需要在 Jetson 平台上测试：

```bash
# 编译
cd build && cmake .. && make

# 运行
./media_plane --config ../config/config.yaml

# 使用测试页面连接
# 打开浏览器访问 http://localhost:8888/test.html
```

### 2. 动态源切换

在 signaling_server.cpp 中实现 switch_source：

```cpp
} else if (type == "switch_source") {
    std::string new_source = msg.value("source", "");
    // 1. 停止当前 peer
    // 2. 创建新 peer 使用新源
    // 3. 重新协商
}
```

### 3. GStreamer 错误处理

在 pipeline_manager.cpp 中添加总线消息处理：

```cpp
GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
gst_bus_add_watch(bus, [](GstBus*, GstMessage* msg, gpointer data) {
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR:
            // 处理错误
            break;
        case GST_MESSAGE_WARNING:
            // 处理警告
            break;
        case GST_MESSAGE_EOS:
            // 处理流结束
            break;
    }
    return TRUE;
}, this);
```

---

## 🔧 配置示例

### 完整配置 (config.yaml)

```yaml
server:
  signaling_port: 8888
  host: "0.0.0.0"
  max_connections: 10
  jwt_secret: ""  # 建议通过 JWT_SECRET 环境变量设置
  require_auth: false  # 开发模式关闭

video:
  default_source: "head_camera"
  sources:
    - name: "head_camera"
      device: "/dev/video0"
      type: "v4l2"
      enabled: true
    - name: "test_pattern"
      device: ""
      type: "test"
      enabled: true
  encoding:
    codec: "h264"
    hardware_encoder: true
    width: 1280
    height: 720
    framerate: 30
    bitrate: 2000
```

### 环境变量

```bash
export JWT_SECRET="shared-secret-with-control-plane"
export REQUIRE_AUTH=true
export MEDIA_PLANE_PORT=8888
export USE_NVENC=true
```

---

## 📊 协议文档

### 信令消息格式

#### 1. 欢迎消息 (服务端 → 客户端)
```json
{
  "type": "welcome",
  "peer_id": "abc123def456",
  "require_auth": false,
  "available_sources": ["head_camera", "test_pattern"]
}
```

#### 2. 认证 (客户端 → 服务端)
```json
{
  "type": "auth",
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

#### 3. 认证成功 (服务端 → 客户端)
```json
{
  "type": "auth_success",
  "user_id": "user123",
  "username": "admin"
}
```

#### 4. 请求流 (客户端 → 服务端)
```json
{
  "type": "request_stream",
  "source": "head_camera"
}
```

#### 5. SDP Offer (服务端 → 客户端)
```json
{
  "type": "offer",
  "sdp": "v=0\r\no=- ..."
}
```

#### 6. SDP Answer (客户端 → 服务端)
```json
{
  "type": "answer",
  "sdp": "v=0\r\no=- ..."
}
```

#### 7. ICE Candidate (双向)
```json
{
  "type": "ice_candidate",
  "candidate": "candidate:...",
  "sdp_mid": "video0",
  "sdp_mline_index": 0
}
```

#### 8. 停止流 (客户端 → 服务端)
```json
{
  "type": "stop_stream"
}
```

#### 9. 流已停止 (服务端 → 客户端)
```json
{
  "type": "stream_stopped"
}
```

#### 10. 获取源 (客户端 → 服务端)
```json
{
  "type": "get_sources"
}
```

#### 11. 源列表 (服务端 → 客户端)
```json
{
  "type": "sources",
  "sources": ["head_camera", "arm_camera", "test_pattern"]
}
```

#### 12. 错误 (服务端 → 客户端)
```json
{
  "type": "error",
  "error": "unauthorized",
  "details": "Authentication required"
}
```

#### 13. Ping/Pong 保活
```json
{"type": "ping", "timestamp": 1705641234567}
{"type": "pong", "timestamp": 1705641234567}
```

#### 14. 切换视频源 (客户端 → 服务端)
```json
{
  "type": "switch_source",
  "source": "arm_camera"
}
```

#### 15. 源切换响应 (服务端 → 客户端)
```json
{
  "type": "source_switched",
  "source": "arm_camera",
  "reconnect_required": true,
  "message": "Please request stream again with new source"
}
```

#### 16. 获取统计 (客户端 → 服务端)
```json
{
  "type": "get_stats"
}
```

#### 17. 统计响应 (服务端 → 客户端)
```json
{
  "type": "stats",
  "connections": 3,
  "active_peers": 2,
  "total_peers_created": 15,
  "error_count": 1,
  "available_sources": ["head_camera", "test_pattern"]
}
```

---

## 📁 旧代码参考

### old_backup 中的相机相关功能

| 旧文件 | 功能 | 迁移状态 |
|--------|------|----------|
| `camera.py` | 相机状态管理 | ✅ 已迁移到 Media Plane |
| `recording.py` | 视频录制 | P2 待实现 |
| `ros2_bridge/camera.py` | ROS2 相机桥接 | ✅ 通过 Data Plane |

---

## 📈 完成度统计

| 类别 | 总数 | 完成 | 完成率 |
|------|------|------|--------|
| 配置系统 | 8 | 8 | 100% |
| 信令服务器 | 14 | 14 | 100% |
| WebRTC Peer | 8 | 7 | 88% |
| Pipeline 管理 | 8 | 8 | 100% |
| 认证 | 4 | 4 | 100% |
| 监控 | 5 | 4 | 80% |
| **总体** | **47** | **45** | **~95%** |

---

## 🎯 下一步计划

1. ~~完善媒体服务器集成~~ ✅
2. ~~添加 JWT 认证~~ ✅
3. ~~动态视频源切换~~ ✅ (需要重连)
4. ~~GStreamer 错误处理~~ ✅ (基本完成)
5. **测试基本流程** - 使用 videotestsrc 测试
6. **延迟统计** - 帧延迟测量
7. **Prometheus 指标** - 可选

---

## 📝 编译和测试

### 编译命令

```bash
cd media_plane
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 测试命令

```bash
# 使用测试视频源
./media_plane_server ../config/config_test.yaml

# 查看 GStreamer 调试信息
GST_DEBUG=3 ./media_plane_server
```

### 前端测试

使用 README.md 中的测试 HTML 页面连接信令服务器。

---

## 🔗 相关文档

- [重构.md](../重构.md) - 架构设计原则
- [重构开发计划.md](../重构开发计划.md) - 整体开发计划
- [Control Plane TODO](../control_plane/TODO.md)
- [Data Plane TODO](../data_plane/TODO.md)

---

*最后更新：2025-01-19*
