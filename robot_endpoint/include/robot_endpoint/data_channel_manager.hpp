#pragma once

#include <vector>
#include <string>
#include <functional>

namespace qyh::robot {

class DataChannelManager {
public:
    using OnMessage = std::function<void(const std::string& channel, const std::vector<uint8_t>& data)>;

    void set_on_message(OnMessage cb);

    void send_control(const std::vector<uint8_t>& data);
    void send_state(const std::vector<uint8_t>& data);
    void send_event(const std::vector<uint8_t>& data);

private:
    OnMessage on_message_;
};

} // namespace qyh::robot
