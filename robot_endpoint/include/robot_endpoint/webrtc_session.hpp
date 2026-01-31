#pragma once

#include "robot_endpoint/signaling_client.hpp"
#include "robot_endpoint/webrtc_peer.hpp"

#include <string>
#include <unordered_map>
#include <memory>
#include <mutex>

#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
namespace rtc {
class PeerConnection;
class Track;
}
#endif

namespace qyh::robot {

class WebRtcSession {
public:
    explicit WebRtcSession(SignalingClient& signaling);

    void on_offer(const SignalingMessage& msg);
    void on_ice(const SignalingMessage& msg);

    // 推送编码帧（H265）到 WebRTC Track（当前为占位）
    void send_video(const std::string& stream_name,
                    const uint8_t* data,
                    size_t size,
                    const std::string& codec);

private:
    SignalingClient& signaling_;
    WebRtcPeer peer_;
    std::string current_session_id_;

#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    std::shared_ptr<rtc::PeerConnection> pc_;
    std::unordered_map<std::string, std::shared_ptr<rtc::Track>> video_tracks_;
    std::mutex track_mutex_;
#endif
};

} // namespace qyh::robot
