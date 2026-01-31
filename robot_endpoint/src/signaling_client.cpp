#include "robot_endpoint/signaling_client.hpp"

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <iostream>
#include <sstream>
#include <chrono>

namespace qyh::robot {

namespace {
    size_t write_callback(void* contents, size_t size, size_t nmemb, void* userp) {
        size_t total = size * nmemb;
        auto* s = static_cast<std::string*>(userp);
        s->append(static_cast<char*>(contents), total);
        return total;
    }

    nlohmann::json message_to_json(const SignalingMessage& msg) {
        nlohmann::json j;
        j["type"] = msg.type;
        if (!msg.session_id.empty()) j["session_id"] = msg.session_id;
        if (!msg.sdp.empty()) j["sdp"] = msg.sdp;
        if (!msg.candidate.empty()) j["candidate"] = msg.candidate;
        if (!msg.sdp_mid.empty()) j["sdp_mid"] = msg.sdp_mid;
        if (msg.sdp_mline_index >= 0) j["sdp_mline_index"] = msg.sdp_mline_index;
        return j;
    }

    SignalingMessage json_to_message(const nlohmann::json& j) {
        SignalingMessage msg;
        msg.type = j.value("type", "");
        msg.session_id = j.value("session_id", "");
        msg.sdp = j.value("sdp", "");
        msg.candidate = j.value("candidate", "");
        msg.sdp_mid = j.value("sdp_mid", "");
        msg.sdp_mline_index = j.value("sdp_mline_index", -1);
        return msg;
    }
}

SignalingClient::SignalingClient(std::string server_url, std::string robot_id, std::string robot_secret)
    : server_url_(std::move(server_url))
    , robot_id_(std::move(robot_id))
    , robot_secret_(std::move(robot_secret)) {
    curl_global_init(CURL_GLOBAL_DEFAULT);
}

bool SignalingClient::connect() {
    std::cout << "[SignalingClient] connect to " << server_url_ << std::endl;
    return true;
}

void SignalingClient::disconnect() {
    std::cout << "[SignalingClient] disconnect" << std::endl;
}

bool SignalingClient::send_offer(const SignalingMessage& msg) {
    return post_message(msg, nullptr, nullptr, 0);
}

bool SignalingClient::send_answer(const SignalingMessage& msg) {
    return post_message(msg, nullptr, nullptr, 0);
}

bool SignalingClient::send_ice_candidate(const SignalingMessage& msg) {
    return post_message(msg, nullptr, nullptr, 0);
}

bool SignalingClient::register_robot(const std::string& name, const std::vector<std::string>& video_sources) {
    nlohmann::json body;
    body["name"] = name;
    body["video_sources"] = video_sources;

    std::string response;

    CURL* curl = curl_easy_init();
    if (!curl) return false;

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    std::string header_id = "X-Robot-Id: " + robot_id_;
    std::string header_secret = "X-Robot-Secret: " + robot_secret_;
    headers = curl_slist_append(headers, header_id.c_str());
    headers = curl_slist_append(headers, header_secret.c_str());

    std::string url = base_url() + "/robots/" + robot_id_ + "/register";

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    auto payload = body.dump();
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, payload.size());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK || http_code != 200) {
        std::cerr << "[SignalingClient] register failed: " << curl_easy_strerror(res)
                  << " code=" << http_code << std::endl;
        return false;
    }

    return true;
}

std::vector<SignalingMessage> SignalingClient::poll(const std::string& session_id) {
    SignalingMessage poll_msg;
    poll_msg.type = "poll";
    poll_msg.session_id = session_id;

    std::string out_session_id;
    std::vector<SignalingMessage> out_messages;
    post_message(poll_msg, &out_session_id, &out_messages, 5000);
    return out_messages;
}

void SignalingClient::start_polling(std::function<void(const SignalingMessage&)> on_message,
                                    int interval_ms,
                                    int timeout_ms) {
    if (polling_) return;
    polling_ = true;
    polling_thread_ = std::make_unique<std::thread>([this, on_message, interval_ms, timeout_ms]() {
        while (polling_) {
            SignalingMessage poll_msg;
            poll_msg.type = "poll";
            poll_msg.session_id = "";

            std::string out_session_id;
            std::vector<SignalingMessage> messages;
            post_message(poll_msg, &out_session_id, &messages, timeout_ms);
            for (const auto& msg : messages) {
                on_message(msg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
        }
    });
}

void SignalingClient::stop_polling() {
    polling_ = false;
    if (polling_thread_ && polling_thread_->joinable()) {
        polling_thread_->join();
    }
    polling_thread_.reset();
}

std::string SignalingClient::base_url() const {
    auto pos = server_url_.rfind("/signaling");
    if (pos == std::string::npos) return server_url_;
    return server_url_.substr(0, pos);
}

bool SignalingClient::post_message(
    const SignalingMessage& msg,
    std::string* out_session_id,
    std::vector<SignalingMessage>* out_messages,
    int timeout_ms
) {
    nlohmann::json body;
    body["robot_id"] = robot_id_;
    if (!msg.session_id.empty()) body["session_id"] = msg.session_id;
    body["message"] = message_to_json(msg);
    if (timeout_ms > 0) body["timeout_ms"] = timeout_ms;

    std::string response;

    CURL* curl = curl_easy_init();
    if (!curl) return false;

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    std::string header_id = "X-Robot-Id: " + robot_id_;
    std::string header_secret = "X-Robot-Secret: " + robot_secret_;
    headers = curl_slist_append(headers, header_id.c_str());
    headers = curl_slist_append(headers, header_secret.c_str());

    curl_easy_setopt(curl, CURLOPT_URL, server_url_.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POST, 1L);
    auto payload = body.dump();
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, payload.size());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

    CURLcode res = curl_easy_perform(curl);
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);

    if (res != CURLE_OK || http_code != 200) {
        std::cerr << "[SignalingClient] HTTP error: " << curl_easy_strerror(res)
                  << " code=" << http_code << std::endl;
        return false;
    }

    if (!out_session_id && !out_messages) {
        return true;
    }

    try {
        auto json = nlohmann::json::parse(response);
        if (out_session_id) *out_session_id = json.value("session_id", "");
        if (out_messages) {
            out_messages->clear();
            for (const auto& item : json.value("messages", nlohmann::json::array())) {
                out_messages->push_back(json_to_message(item));
            }
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[SignalingClient] JSON parse error: " << e.what() << std::endl;
        return false;
    }
}

} // namespace qyh::robot
