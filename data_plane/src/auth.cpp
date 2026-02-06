/**
 * @file auth.cpp
 * @brief JWT 认证实现
 */

#include "data_plane/auth.hpp"
#include "data_plane/logger.hpp"

#include <openssl/hmac.h>
#include <openssl/evp.h>
#include <nlohmann/json.hpp>

#include <cstring>
#include <sstream>
#include <vector>
#include <chrono>

namespace qyh::dataplane {

JWTValidator::JWTValidator(const std::string& secret,
                           const std::string& algorithm,
                           const std::string& audience,
                           const std::string& reject_scope)
    : secret_(secret)
    , algorithm_(algorithm)
    , audience_(audience)
    , reject_scope_(reject_scope)
{
}

std::optional<SessionUserInfo> JWTValidator::validate(const std::string& token) {
    auto first_dot = token.find('.');
    auto second_dot = token.find('.', first_dot + 1);
    
    if (first_dot == std::string::npos || second_dot == std::string::npos) {
        return std::nullopt;
    }
    
    std::string header_b64 = token.substr(0, first_dot);
    std::string payload_b64 = token.substr(first_dot + 1, second_dot - first_dot - 1);
    std::string signature_b64 = token.substr(second_dot + 1);
    
    std::string header_payload = header_b64 + "." + payload_b64;
    if (!verify_signature(header_payload, signature_b64)) {
        return std::nullopt;
    }
    std::string payload_json = base64url_decode(payload_b64);
    auto result = parse_payload(payload_json);
    if (!result) {
        return std::nullopt;
    }
    if (!verify_claims(*result)) {
        return std::nullopt;
    }
    return result;
}

bool JWTValidator::is_expired(const std::string& token) {
    auto first_dot = token.find('.');
    auto second_dot = token.find('.', first_dot + 1);
    
    if (first_dot == std::string::npos || second_dot == std::string::npos) {
        return true;
    }
    
    std::string payload_b64 = token.substr(first_dot + 1, second_dot - first_dot - 1);
    std::string payload_json = base64url_decode(payload_b64);
    
    try {
        auto json = nlohmann::json::parse(payload_json);
        if (json.contains("exp") && json["exp"].is_number()) {
            int64_t exp = json["exp"].get<int64_t>();
            auto now = std::chrono::system_clock::now();
            auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(
                now.time_since_epoch()).count();
            return now_sec > exp;
        }
    } catch (const nlohmann::json::exception& e) {
        LOG_ERROR("Failed to parse JWT payload: " << e.what());
    }
    return true;
}

std::string JWTValidator::base64url_decode(const std::string& input) {
    std::string base64 = input;
    std::replace(base64.begin(), base64.end(), '-', '+');
    std::replace(base64.begin(), base64.end(), '_', '/');
    while (base64.size() % 4 != 0) {
        base64 += '=';
    }
    
    static const std::string chars = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    
    std::string result;
    std::vector<int> T(256, -1);
    for (size_t i = 0; i < chars.size(); ++i) {
        T[chars[i]] = static_cast<int>(i);
    }
    
    int val = 0, valb = -8;
    for (unsigned char c : base64) {
        if (T[c] == -1) break;
        val = (val << 6) + T[c];
        valb += 6;
        if (valb >= 0) {
            result.push_back(static_cast<char>((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    return result;
}

bool JWTValidator::verify_signature(const std::string& header_payload, 
                                     const std::string& signature) {
    if (algorithm_ != "HS256") {
        return false;
    }
    
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int digest_len = 0;
    
    HMAC(EVP_sha256(),
         secret_.c_str(), static_cast<int>(secret_.size()),
         reinterpret_cast<const unsigned char*>(header_payload.c_str()),
         header_payload.size(),
         digest, &digest_len);
    
    static const char* chars = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
    
    std::string computed;
    int val = 0, valb = -6;
    for (unsigned int i = 0; i < digest_len; ++i) {
        val = (val << 8) + digest[i];
        valb += 8;
        while (valb >= 0) {
            computed.push_back(chars[(val >> valb) & 0x3F]);
            valb -= 6;
        }
    }
    if (valb > -6) {
        computed.push_back(chars[((val << 8) >> (valb + 8)) & 0x3F]);
    }
    while (!computed.empty() && computed.back() == '=') {
        computed.pop_back();
    }
    
    return computed == signature;
}

std::optional<SessionUserInfo> JWTValidator::parse_payload(const std::string& payload_json) {
    SessionUserInfo info;
    
    try {
        auto json = nlohmann::json::parse(payload_json);
        
        // 支持 "sub" 和 "user_id" 两种字段
        if (json.contains("sub")) {
            if (json["sub"].is_string()) {
                info.user_id = std::stoll(json["sub"].get<std::string>());
            } else if (json["sub"].is_number()) {
                info.user_id = json["sub"].get<int64_t>();
            }
        } else if (json.contains("user_id")) {
            if (json["user_id"].is_string()) {
                info.user_id = std::stoll(json["user_id"].get<std::string>());
            } else if (json["user_id"].is_number()) {
                info.user_id = json["user_id"].get<int64_t>();
            }
        }
        
        if (json.contains("username") && json["username"].is_string()) {
            info.username = json["username"].get<std::string>();
        }
        if (json.contains("role") && json["role"].is_string()) {
            info.role = json["role"].get<std::string>();
        }
        if (json.contains("aud")) {
            if (json["aud"].is_string()) {
                info.audience = json["aud"].get<std::string>();
            } else if (json["aud"].is_array() && !json["aud"].empty()) {
                info.audience = json["aud"][0].get<std::string>();
            }
        }
        if (json.contains("scope") && json["scope"].is_string()) {
            info.scope = json["scope"].get<std::string>();
        }
        if (json.contains("permissions") && json["permissions"].is_array()) {
            for (const auto& perm : json["permissions"]) {
                if (perm.is_string()) {
                    info.permissions.push_back(perm.get<std::string>());
                }
            }
        }
    } catch (const nlohmann::json::exception& e) {
        LOG_ERROR("Failed to parse JWT payload: " << e.what());
        return std::nullopt;
    }
    
    if (info.user_id == 0) {
        return std::nullopt;
    }
    return info;
}

bool JWTValidator::verify_claims(const SessionUserInfo& info) const {
    if (!audience_.empty() && !info.audience.empty() && info.audience != audience_) {
        return false;
    }

    if (!reject_scope_.empty() && !info.scope.empty()) {
        if (info.scope.find(reject_scope_) != std::string::npos) {
            return false;
        }
    }

    return true;
}

} // namespace qyh::dataplane
