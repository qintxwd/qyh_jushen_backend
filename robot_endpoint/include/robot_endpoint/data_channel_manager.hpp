#pragma once

#include <vector>
#include <string>
#include <functional>
#include <memory>

#include "control_channel.pb.h"
#include "state_channel.pb.h"
#include "event.pb.h"

#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
namespace rtc {
class PeerConnection;
class DataChannel;
}
#endif

namespace qyh::robot {

class DataChannelManager {
public:
    using OnControl = std::function<void(const qyh::dataplane::ControlChannelMessage&)>;
    using OnState = std::function<void(const qyh::dataplane::StateChannelMessage&)>;
    using OnEvent = std::function<void(const qyh::event::EventChannelMessage&)>;

    void set_on_control(OnControl cb);
    void set_on_state(OnState cb);
    void set_on_event(OnEvent cb);

#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    void attach_peer(std::shared_ptr<rtc::PeerConnection> pc);
    void setup_channels();
#endif

    void send_control(const qyh::dataplane::ControlChannelMessage& msg);
    void send_state(const qyh::dataplane::StateChannelMessage& msg);
    void send_event(const qyh::event::EventChannelMessage& msg);

private:
#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    std::shared_ptr<rtc::PeerConnection> pc_;
    std::shared_ptr<rtc::DataChannel> control_;
    std::shared_ptr<rtc::DataChannel> state_;
    std::shared_ptr<rtc::DataChannel> event_;
#endif

    OnControl on_control_;
    OnState on_state_;
    OnEvent on_event_;
};

} // namespace qyh::robot
