#pragma once

#include "robot_endpoint/config.hpp"

#include <functional>
#include <memory>

#include "control_channel.pb.h"
#include "state_channel.pb.h"

namespace qyh::robot {

class Ros2ControlStateBridge {
public:
    using StateCallback = std::function<void(const qyh::dataplane::StateChannelMessage&)>;

    explicit Ros2ControlStateBridge(const Ros2Config& cfg);
    ~Ros2ControlStateBridge();

    void set_on_state(StateCallback cb);
    void handle_control(const qyh::dataplane::ControlChannelMessage& msg);

    void start();
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace qyh::robot