#include "robot_endpoint/ros2_bridge.hpp"

#include <iostream>
#include <thread>

#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#endif

namespace qyh::robot {

struct Ros2Bridge::Impl {
    std::vector<CameraConfig> cameras;
    ImageCallback on_color;
    ImageCallback on_depth;

#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<rclcpp::Node> node;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs;
    std::unique_ptr<std::thread> spin_thread;
#endif
};

Ros2Bridge::Ros2Bridge(const std::vector<CameraConfig>& cameras)
    : impl_(std::make_unique<Impl>()) {
    impl_->cameras = cameras;
}

Ros2Bridge::~Ros2Bridge() {
    stop();
}

void Ros2Bridge::set_on_color(ImageCallback cb) {
    impl_->on_color = std::move(cb);
}

void Ros2Bridge::set_on_depth(ImageCallback cb) {
    impl_->on_depth = std::move(cb);
}

void Ros2Bridge::start() {
#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    if (rclcpp::ok()) {
        impl_->node = std::make_shared<rclcpp::Node>("qyh_robot_endpoint");
        for (const auto& cam : impl_->cameras) {
            if (!cam.color_topic.empty()) {
                auto sub = impl_->node->create_subscription<sensor_msgs::msg::Image>(
                    cam.color_topic, rclcpp::SensorDataQoS(),
                    [this, cam](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                        if (impl_->on_color) {
                            impl_->on_color(cam, msg->data.data(), msg->data.size(),
                                            msg->encoding, msg->width, msg->height);
                        }
                    }
                );
                impl_->subs.push_back(sub);
            }
            if (!cam.depth_topic.empty()) {
                auto sub = impl_->node->create_subscription<sensor_msgs::msg::Image>(
                    cam.depth_topic, rclcpp::SensorDataQoS(),
                    [this, cam](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                        if (impl_->on_depth) {
                            impl_->on_depth(cam, msg->data.data(), msg->data.size(),
                                            msg->encoding, msg->width, msg->height);
                        }
                    }
                );
                impl_->subs.push_back(sub);
            }
        }

        impl_->executor.add_node(impl_->node);
        impl_->spin_thread = std::make_unique<std::thread>([this]() {
            impl_->executor.spin();
        });
        std::cout << "[Ros2Bridge] started" << std::endl;
        return;
    }
#endif
    std::cout << "[Ros2Bridge] ROS2 disabled (build without ROBOT_ENDPOINT_ENABLE_ROS2)" << std::endl;
}

void Ros2Bridge::stop() {
#ifdef ROBOT_ENDPOINT_ENABLE_ROS2
    if (impl_->node) {
        impl_->executor.cancel();
        if (impl_->spin_thread && impl_->spin_thread->joinable()) {
            impl_->spin_thread->join();
        }
        impl_->spin_thread.reset();
        impl_->subs.clear();
        impl_->node.reset();
        std::cout << "[Ros2Bridge] stopped" << std::endl;
    }
#endif
}

} // namespace qyh::robot
