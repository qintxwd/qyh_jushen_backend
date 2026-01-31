#include <iostream>
#include <thread>
#include <chrono>

#include "robot_endpoint/signaling_client.hpp"
#include "robot_endpoint/webrtc_peer.hpp"
#include "robot_endpoint/config.hpp"

int main(int argc, char** argv) {
    std::cout << "QYH Robot Endpoint (skeleton)" << std::endl;

    std::string config_path = "config/config.yaml";
    if (argc > 1) {
        config_path = argv[1];
    }

    auto cfg = qyh::robot::load_config(config_path);

    qyh::robot::SignalingClient client(
        cfg.signaling.server_url,
        cfg.robot.id,
        cfg.signaling.robot_secret
    );
    client.connect();

    client.register_robot(cfg.robot.name, {"main", "arm"});

    qyh::robot::WebRtcPeer peer;

    client.start_polling([&client, &peer](const qyh::robot::SignalingMessage& msg) {
        if (msg.type == "offer") {
            auto answer_sdp = peer.handle_remote_offer(msg.sdp);
            qyh::robot::SignalingMessage answer;
            answer.type = "answer";
            answer.session_id = msg.session_id;
            answer.sdp = answer_sdp;
            client.send_answer(answer);
        } else if (msg.type == "ice_candidate") {
            std::cout << "[Robot] ICE candidate received for session " << msg.session_id << std::endl;
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(2));
    client.stop_polling();

    return 0;
}
