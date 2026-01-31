#pragma once

#include "robot_endpoint/config.hpp"

#include <functional>
#include <memory>
#include <vector>
#include <string>

namespace qyh::robot {

class Ros2Bridge {
public:
    using ImageCallback = std::function<void(const CameraConfig& cam,
                                             const uint8_t* data,
                                             size_t size,
                                             const std::string& encoding,
                                             int width,
                                             int height)>;

    Ros2Bridge(const std::vector<CameraConfig>& cameras);
    ~Ros2Bridge();

    void set_on_color(ImageCallback cb);
    void set_on_depth(ImageCallback cb);

    void start();
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace qyh::robot
