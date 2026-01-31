#include "robot_endpoint/data_channel_manager.hpp"

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
