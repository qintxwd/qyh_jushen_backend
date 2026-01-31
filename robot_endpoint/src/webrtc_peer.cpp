#include "robot_endpoint/webrtc_peer.hpp"

#include <iostream>

namespace qyh::robot {

WebRtcPeer::WebRtcPeer() = default;

void WebRtcPeer::set_on_local_answer(OnLocalAnswer cb) {
    on_local_answer_ = std::move(cb);
}

void WebRtcPeer::set_on_local_ice(OnLocalIce cb) {
    on_local_ice_ = std::move(cb);
}

std::string WebRtcPeer::handle_remote_offer(const std::string& sdp) {
    std::cout << "[WebRtcPeer] received offer, size=" << sdp.size() << std::endl;

    // TODO: 使用真实 WebRTC 生成 Answer
    std::string answer = "v=0\r\n";
    if (on_local_answer_) {
        on_local_answer_(answer);
    }
    return answer;
}

void WebRtcPeer::handle_remote_ice(const IceCandidate& cand) {
    std::cout << "[WebRtcPeer] received ICE candidate, mid=" << cand.sdp_mid << std::endl;
}

} // namespace qyh::robot
