#pragma once

#include <string>
#include <functional>

namespace qyh::robot {

struct IceCandidate {
    std::string candidate;
    std::string sdp_mid;
    int sdp_mline_index = -1;
};

class WebRtcPeer {
public:
    using OnLocalAnswer = std::function<void(const std::string& sdp)>;
    using OnLocalIce = std::function<void(const IceCandidate& cand)>;

    WebRtcPeer();

    void set_on_local_answer(OnLocalAnswer cb);
    void set_on_local_ice(OnLocalIce cb);

    // 输入远端 Offer，生成 Answer（后续接入真实 WebRTC）
    std::string handle_remote_offer(const std::string& sdp);

    // 输入远端 ICE
    void handle_remote_ice(const IceCandidate& cand);

private:
    OnLocalAnswer on_local_answer_;
    OnLocalIce on_local_ice_;
};

} // namespace qyh::robot
