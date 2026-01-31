#pragma once

#include <string>
#include <functional>
#include <thread>
#include <atomic>

namespace qyh::robot {

class MediaPipeline {
public:
    MediaPipeline();
    ~MediaPipeline();

    // 使用 Jetson 硬件编码器
    bool start(const std::string& device = "/dev/video0",
               int width = 1280,
               int height = 720,
               int fps = 30,
               int bitrate = 4000000,
               const std::string& encoding = "h265");

    bool start_appsrc(const std::string& name,
                      int width,
                      int height,
                      int fps,
                      int bitrate,
                      const std::string& encoding = "h265");

    bool push_frame(const uint8_t* data,
                    size_t size,
                    int width,
                    int height,
                    const std::string& encoding);

    void set_on_encoded(std::function<void(const uint8_t* data, size_t size, const std::string& codec)> cb);

    void stop();

private:
    void start_pull_thread(const std::string& codec);
    void stop_pull_thread();

    void* appsrc_ = nullptr;
    void* appsink_ = nullptr;
    std::string last_caps_format_;
    void* pipeline_ = nullptr;

    std::function<void(const uint8_t* data, size_t size, const std::string& codec)> on_encoded_;
    std::atomic<bool> pulling_{false};
    std::unique_ptr<std::thread> pull_thread_;
};

} // namespace qyh::robot
