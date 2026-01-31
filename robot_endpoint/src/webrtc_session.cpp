#include "robot_endpoint/webrtc_session.hpp"

#include <iostream>

#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
#include <rtc/rtc.hpp>
#endif

namespace qyh::robot {

WebRtcSession::WebRtcSession(SignalingClient& signaling)
    : signaling_(signaling) {
#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    rtc::Configuration config;
    pc_ = std::make_shared<rtc::PeerConnection>(config);

    pc_->onLocalDescription([this](rtc::Description desc) {
        if (desc.typeString() == "answer" && !current_session_id_.empty()) {
            SignalingMessage answer;
            answer.type = "answer";
            answer.session_id = current_session_id_;
            answer.sdp = std::string(desc);
            signaling_.send_answer(answer);
        }
    });

    pc_->onLocalCandidate([this](rtc::Candidate cand) {
        if (current_session_id_.empty()) return;
        SignalingMessage ice;
        ice.type = "ice_candidate";
        ice.session_id = current_session_id_;
        ice.candidate = cand.candidate();
        ice.sdp_mid = cand.mid();
        ice.sdp_mline_index = cand.mlineindex();
        signaling_.send_ice_candidate(ice);
    });
#endif
}

void WebRtcSession::set_data_channels(std::shared_ptr<DataChannelManager> channels) {
    data_channels_ = std::move(channels);
#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    if (pc_ && data_channels_) {
        data_channels_->attach_peer(pc_);
        data_channels_->setup_channels();
    }
#endif
}

void WebRtcSession::on_offer(const SignalingMessage& msg) {
    current_session_id_ = msg.session_id;
#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    if (pc_) {
        rtc::Description offer(msg.sdp, "offer");
        pc_->setRemoteDescription(offer);
        pc_->setLocalDescription();
        return;
    }
#endif
    auto answer_sdp = peer_.handle_remote_offer(msg.sdp);
    SignalingMessage answer;
    answer.type = "answer";
    answer.session_id = current_session_id_;
    answer.sdp = answer_sdp;
    signaling_.send_answer(answer);
}

void WebRtcSession::on_ice(const SignalingMessage& msg) {
#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    if (pc_) {
        rtc::Candidate cand(msg.candidate, msg.sdp_mid, msg.sdp_mline_index);
        pc_->addRemoteCandidate(cand);
        return;
    }
#endif
    WebRtcPeer::IceCandidate cand;
    cand.candidate = msg.candidate;
    cand.sdp_mid = msg.sdp_mid;
    cand.sdp_mline_index = msg.sdp_mline_index;
    peer_.handle_remote_ice(cand);
}

void WebRtcSession::send_video(const std::string& stream_name,
                               const uint8_t* data,
                               size_t size,
                               const std::string& codec) {
#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    if (!pc_) return;
    if (codec != "h265" && codec != "H265") return;

    std::shared_ptr<rtc::Track> track;
    {
        std::lock_guard<std::mutex> lock(track_mutex_);
        auto it = video_tracks_.find(stream_name);
        if (it != video_tracks_.end()) {
            track = it->second;
        } else {
            rtc::Description::Video desc(stream_name, rtc::Description::Direction::SendOnly);
            desc.addH265Codec(96); // dynamic payload type
            track = pc_->addTrack(desc);

            auto packetizer = std::make_shared<rtc::H265RtpPacketizer>(
                rtc::H265RtpPacketizer::Configuration{96});
            packetizer->setMaxPacketSize(1200);
            track->setMediaHandler(packetizer);

            video_tracks_[stream_name] = track;
        }
    }

    if (!track) return;

    rtc::binary sample(data, data + size);
    track->send(sample);
    return;
#endif
    std::cout << "[WebRtcSession] stream=" << stream_name
              << " codec=" << codec
              << " bytes=" << size << std::endl;
    (void)data;
}

} // namespace qyh::robot
