#pragma once

#include <string>
#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <memory>

namespace qyh::robot {

struct SignalingMessage {
    // type: "offer" | "answer" | "ice_candidate" | "poll"
    std::string type;
    std::string session_id;
    std::string sdp;
    std::string candidate;
    std::string sdp_mid;
    int sdp_mline_index = -1;
};

class SignalingClient {
public:
    SignalingClient(std::string server_url, std::string robot_id, std::string robot_secret);

    bool connect();
    void disconnect();

    bool send_offer(const SignalingMessage& msg);
    bool send_answer(const SignalingMessage& msg);
    bool send_ice_candidate(const SignalingMessage& msg);

    bool register_robot(const std::string& name, const std::vector<std::string>& video_sources);

    // 轮询获取需要下发给机器人的信令消息
    std::vector<SignalingMessage> poll(const std::string& session_id);

    // 后台轮询
    void start_polling(std::function<void(const SignalingMessage&)> on_message,
                       int interval_ms = 200,
                       int timeout_ms = 5000);
    void stop_polling();

private:
    std::string base_url() const;
    bool post_message(
        const SignalingMessage& msg,
        std::string* out_session_id,
        std::vector<SignalingMessage>* out_messages,
        int timeout_ms
    );

    std::string server_url_;
    std::string robot_id_;
    std::string robot_secret_;

    std::atomic<bool> polling_{false};
    std::unique_ptr<std::thread> polling_thread_;
};

} // namespace qyh::robot
