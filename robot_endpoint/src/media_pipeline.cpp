#include "robot_endpoint/media_pipeline.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <iostream>
#include <sstream>
#include <mutex>
#include <cstring>

namespace qyh::robot {

MediaPipeline::MediaPipeline() {
    gst_init(nullptr, nullptr);
}

MediaPipeline::~MediaPipeline() {
    stop();
}

bool MediaPipeline::start(const std::string& device, int width, int height, int fps, int bitrate, const std::string& encoding) {
    stop();

    std::string enc = (encoding == "h264") ? "nvv4l2h264enc" : "nvv4l2h265enc";
    std::string parse = (encoding == "h264") ? "h264parse" : "h265parse";

    std::ostringstream pipeline_desc;
    pipeline_desc
        << "v4l2src device=" << device << " ! "
        << "video/x-raw,width=" << width << ",height=" << height << ",framerate=" << fps << "/1 ! "
        << "nvvidconv ! "
        << enc << " bitrate=" << bitrate << " ! "
        << parse << " ! "
        << "appsink name=appsink";

    GError* error = nullptr;
    auto* pipeline = gst_parse_launch(pipeline_desc.str().c_str(), &error);
    if (!pipeline || error) {
        std::cerr << "[MediaPipeline] failed to create pipeline: "
                  << (error ? error->message : "unknown") << std::endl;
        if (error) g_error_free(error);
        if (pipeline) gst_object_unref(pipeline);
        return false;
    }

    pipeline_ = pipeline;
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline), "appsink");
    gst_element_set_state(static_cast<GstElement*>(pipeline_), GST_STATE_PLAYING);

    start_pull_thread(encoding);

    std::cout << "[MediaPipeline] started: " << pipeline_desc.str() << std::endl;
    return true;
}

bool MediaPipeline::start_appsrc(const std::string& name,
                                 int width,
                                 int height,
                                 int fps,
                                 int bitrate,
                                 const std::string& encoding) {
    stop();

    std::string enc = (encoding == "h264") ? "nvv4l2h264enc" : "nvv4l2h265enc";
    std::string parse = (encoding == "h264") ? "h264parse" : "h265parse";

    std::ostringstream pipeline_desc;
    pipeline_desc
        << "appsrc name=" << name << " is-live=true block=true format=3 do-timestamp=true ! "
        << "video/x-raw,width=" << width << ",height=" << height;
    if (fps > 0) {
        pipeline_desc << ",framerate=" << fps << "/1";
    }
    pipeline_desc << " ! "
        << "nvvidconv ! "
        << enc << " bitrate=" << bitrate << " ! "
        << parse << " ! "
        << "appsink name=appsink";

    GError* error = nullptr;
    auto* pipeline = gst_parse_launch(pipeline_desc.str().c_str(), &error);
    if (!pipeline || error) {
        std::cerr << "[MediaPipeline] failed to create appsrc pipeline: "
                  << (error ? error->message : "unknown") << std::endl;
        if (error) g_error_free(error);
        if (pipeline) gst_object_unref(pipeline);
        return false;
    }

    pipeline_ = pipeline;
    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline), name.c_str());
    if (!appsrc_) {
        std::cerr << "[MediaPipeline] appsrc not found: " << name << std::endl;
    }
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline), "appsink");
    gst_element_set_state(static_cast<GstElement*>(pipeline_), GST_STATE_PLAYING);

    start_pull_thread(encoding);

    std::cout << "[MediaPipeline] appsrc started: " << pipeline_desc.str() << std::endl;
    return true;
}

namespace {
    std::string map_encoding_to_gst(const std::string& enc) {
        if (enc == "rgb8") return "RGB";
        if (enc == "bgr8") return "BGR";
        if (enc == "mono8") return "GRAY8";
        if (enc == "mono16") return "GRAY16_LE";
        if (enc == "16UC1") return "GRAY16_LE";
        if (enc == "yuv422") return "YUY2";
        if (enc == "nv12") return "NV12";
        return "RGB";
    }
}

bool MediaPipeline::push_frame(const uint8_t* data,
                               size_t size,
                               int width,
                               int height,
                               const std::string& encoding) {
    if (!appsrc_ || !pipeline_) return false;

    std::string gst_format = map_encoding_to_gst(encoding);
    if (gst_format != last_caps_format_) {
        GstCaps* caps = gst_caps_new_simple(
            "video/x-raw",
            "format", G_TYPE_STRING, gst_format.c_str(),
            "width", G_TYPE_INT, width,
            "height", G_TYPE_INT, height,
            NULL
        );
        gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps);
        gst_caps_unref(caps);
        last_caps_format_ = gst_format;
    }

    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    if (!buffer) return false;

    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        std::memcpy(map.data, data, size);
        gst_buffer_unmap(buffer, &map);
    }

    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
    if (ret != GST_FLOW_OK) {
        std::cerr << "[MediaPipeline] push buffer failed" << std::endl;
        return false;
    }
    return true;
}

void MediaPipeline::set_on_encoded(std::function<void(const uint8_t* data, size_t size, const std::string& codec)> cb) {
    on_encoded_ = std::move(cb);
}

void MediaPipeline::start_pull_thread(const std::string& codec) {
    if (!appsink_) return;
    if (pulling_) return;
    pulling_ = true;

    auto* sink = GST_APP_SINK(appsink_);
    gst_app_sink_set_emit_signals(sink, FALSE);
    gst_app_sink_set_drop(sink, TRUE);
    gst_app_sink_set_max_buffers(sink, 5);

    pull_thread_ = std::make_unique<std::thread>([this, codec]() {
        while (pulling_) {
            if (!appsink_) break;
            GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 100000000); // 100ms
            if (!sample) continue;

            GstBuffer* buffer = gst_sample_get_buffer(sample);
            if (buffer) {
                GstMapInfo map;
                if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                    if (on_encoded_) {
                        on_encoded_(map.data, map.size, codec);
                    }
                    gst_buffer_unmap(buffer, &map);
                }
            }
            gst_sample_unref(sample);
        }
    });
}

void MediaPipeline::stop_pull_thread() {
    pulling_ = false;
    if (pull_thread_ && pull_thread_->joinable()) {
        pull_thread_->join();
    }
    pull_thread_.reset();
}

void MediaPipeline::stop() {
    if (!pipeline_) return;
    stop_pull_thread();
    auto* pipeline = static_cast<GstElement*>(pipeline_);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    if (appsrc_) {
        gst_object_unref(appsrc_);
        appsrc_ = nullptr;
    }
    if (appsink_) {
        gst_object_unref(appsink_);
        appsink_ = nullptr;
    }
    pipeline_ = nullptr;
}

} // namespace qyh::robot
