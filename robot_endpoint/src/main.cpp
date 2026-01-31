#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <unordered_map>

#include "robot_endpoint/signaling_client.hpp"
#include "robot_endpoint/webrtc_session.hpp"
#include "robot_endpoint/config.hpp"
#include "robot_endpoint/media_pipeline.hpp"
#include "robot_endpoint/ros2_bridge.hpp"

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

    std::vector<std::string> sources;
    for (const auto& cam : cfg.media.cameras) {
        sources.push_back("color_" + cam.name);
        if (!cam.depth_topic.empty()) {
            sources.push_back("depth_" + cam.name);
        }
    }
    client.register_robot(cfg.robot.name, sources);

    qyh::robot::WebRtcSession webrtc(client);

    std::unordered_map<std::string, std::unique_ptr<qyh::robot::MediaPipeline>> color_pipelines;
    std::unordered_map<std::string, std::unique_ptr<qyh::robot::MediaPipeline>> depth_pipelines;
    for (const auto& cam : cfg.media.cameras) {
        std::cout << "[Media] camera=" << cam.name
                  << " color_topic=" << cam.color_topic
                  << " depth_topic=" << cam.depth_topic << std::endl;

        auto color = std::make_unique<qyh::robot::MediaPipeline>();
        color->set_on_encoded([&](const uint8_t* data, size_t size, const std::string& codec) {
            webrtc.send_video("color_" + cam.name, data, size, codec);
        });
        color->start_appsrc(
            "color_" + cam.name,
            cam.width,
            cam.height,
            cam.fps,
            4000000,
            cfg.media.encoding
        );
        color_pipelines.emplace(cam.name, std::move(color));

        if (!cam.depth_topic.empty()) {
            auto depth = std::make_unique<qyh::robot::MediaPipeline>();
            depth->set_on_encoded([&](const uint8_t* data, size_t size, const std::string& codec) {
                webrtc.send_video("depth_" + cam.name, data, size, codec);
            });
            depth->start_appsrc(
                "depth_" + cam.name,
                cam.width,
                cam.height,
                cam.fps,
                4000000,
                cfg.media.encoding
            );
            depth_pipelines.emplace(cam.name, std::move(depth));
        }
    }

    qyh::robot::Ros2Bridge ros_bridge(cfg.media.cameras);
    ros_bridge.set_on_color([&](const qyh::robot::CameraConfig& cam,
                                const uint8_t* data,
                                size_t size,
                                const std::string& encoding,
                                int width,
                                int height) {
        auto it = color_pipelines.find(cam.name);
        if (it != color_pipelines.end()) {
            it->second->push_frame(data, size, width, height, encoding);
        }
    });
    ros_bridge.set_on_depth([&](const qyh::robot::CameraConfig& cam,
                                const uint8_t* data,
                                size_t size,
                                const std::string& encoding,
                                int width,
                                int height) {
        auto it = depth_pipelines.find(cam.name);
        if (it != depth_pipelines.end()) {
            it->second->push_frame(data, size, width, height, encoding);
        }
    });
    ros_bridge.start();

    client.start_polling([&webrtc](const qyh::robot::SignalingMessage& msg) {
        if (msg.type == "offer") {
            webrtc.on_offer(msg);
        } else if (msg.type == "ice_candidate") {
            webrtc.on_ice(msg);
        }
    });

    std::this_thread::sleep_for(std::chrono::seconds(2));
    client.stop_polling();

    ros_bridge.stop();
    for (auto& [_, p] : color_pipelines) {
        p->stop();
    }
    for (auto& [_, p] : depth_pipelines) {
        p->stop();
    }

    return 0;
}
