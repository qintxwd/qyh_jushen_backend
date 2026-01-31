#include "robot_endpoint/data_channel_manager.hpp"

#include <utility>

#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
#include <rtc/rtc.hpp>
#endif

namespace qyh::robot {

void DataChannelManager::set_on_control(OnControl cb) { on_control_ = std::move(cb); }
void DataChannelManager::set_on_state(OnState cb) { on_state_ = std::move(cb); }
void DataChannelManager::set_on_event(OnEvent cb) { on_event_ = std::move(cb); }

#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
void DataChannelManager::attach_peer(std::shared_ptr<rtc::PeerConnection> pc) {
    pc_ = std::move(pc);
}

void DataChannelManager::setup_channels() {
    if (!pc_) {
        return;
    }

    control_ = pc_->createDataChannel("control");
    state_ = pc_->createDataChannel("state");
    event_ = pc_->createDataChannel("event");

    if (control_) {
        control_->onMessage([this](rtc::message_variant msg) {
            if (!on_control_) {
                return;
            }
            if (const auto *bin = std::get_if<rtc::binary>(&msg)) {
                qyh::dataplane::ControlChannelMessage parsed;
                if (parsed.ParseFromArray(bin->data(), static_cast<int>(bin->size()))) {
                    on_control_(parsed);
                }
            }
        });
    }

    if (state_) {
        state_->onMessage([this](rtc::message_variant msg) {
            if (!on_state_) {
                return;
            }
            if (const auto *bin = std::get_if<rtc::binary>(&msg)) {
                qyh::dataplane::StateChannelMessage parsed;
                if (parsed.ParseFromArray(bin->data(), static_cast<int>(bin->size()))) {
                    on_state_(parsed);
                }
            }
        });
    }

    if (event_) {
        event_->onMessage([this](rtc::message_variant msg) {
            if (!on_event_) {
                return;
            }
            if (const auto *bin = std::get_if<rtc::binary>(&msg)) {
                qyh::event::EventChannelMessage parsed;
                if (parsed.ParseFromArray(bin->data(), static_cast<int>(bin->size()))) {
                    on_event_(parsed);
                }
            }
        });
    }
}
#endif

void DataChannelManager::send_control(const qyh::dataplane::ControlChannelMessage& msg) {
#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    if (!control_) {
        return;
    }
    std::string out;
    if (!msg.SerializeToString(&out)) {
        return;
    }
    control_->send(reinterpret_cast<const std::byte*>(out.data()), out.size());
#else
    (void)msg;
#endif
}

void DataChannelManager::send_state(const qyh::dataplane::StateChannelMessage& msg) {
#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    if (!state_) {
        return;
    }
    std::string out;
    if (!msg.SerializeToString(&out)) {
        return;
    }
    state_->send(reinterpret_cast<const std::byte*>(out.data()), out.size());
#else
    (void)msg;
#endif
}

void DataChannelManager::send_event(const qyh::event::EventChannelMessage& msg) {
#ifdef ROBOT_ENDPOINT_ENABLE_WEBRTC
    if (!event_) {
        return;
    }
    std::string out;
    if (!msg.SerializeToString(&out)) {
        return;
    }
    event_->send(reinterpret_cast<const std::byte*>(out.data()), out.size());
#else
    (void)msg;
#endif
}

} // namespace qyh::robot#include "robot_endpoint/data_channel_manager.hpp"

#include <iostream>

namespace qyh::robot {

void DataChannelManager::set_on_message(OnMessage cb) {
    on_message_ = std::move(cb);
}

void DataChannelManager::send_control(const std::vector<uint8_t>& data) {
    std::cout << "[DataChannel] send control bytes=" << data.size() << std::endl;
}

void DataChannelManager::send_state(const std::vector<uint8_t>& data) {
    std::cout << "[DataChannel] send state bytes=" << data.size() << std::endl;
}

void DataChannelManager::send_event(const std::vector<uint8_t>& data) {
    std::cout << "[DataChannel] send event bytes=" << data.size() << std::endl;
}

} // namespace qyh::robot
