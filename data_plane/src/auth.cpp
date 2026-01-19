/**
 * @file auth.cpp
 * @brief JWT 认证实现
 */

#include "data_plane/auth.hpp"

#include <openssl/hmac.h>
#include <openssl/evp.h>

#include <cstring>
#include <sstream>
#include <vector>
#include <chrono>

// 简单的 JSON 解析（实际项目建议使用 nlohmann/json）
#include <regex>

namespace qyh::dataplane {

JWTValidator::JWTValidator(const std::string& secret, const std::string& algorithm)
    : secret_(secret)
    , algorithm_(algorithm)
{
}

std::optional<UserInfo> JWTValidator::validate(const std::string& token) {
    // JWT 格式: header.payload.signature
    auto first_dot = token.find('.');
    auto second_dot = token.find('.', first_dot + 1);
    
    if (first_dot == std::string::npos || second_dot == std::string::npos) {
        return std::nullopt;
    }
    
    std::string header_b64 = token.substr(0, first_dot);
    std::string payload_b64 = token.substr(first_dot + 1, second_dot - first_dot - 1);
    std::string signature_b64 = token.substr(second_dot + 1);
    
    // 验证签名
    std::string header_payload = header_b64 + "." + payload_b64;
    if (!verify_signature(header_payload, signature_b64)) {
        return std::nullopt;
    }
    
    // 解码 payload
    std::string payload_json = base64url_decode(payload_b64);
    
    // 解析用户信息
    return parse_payload(payload_json);
}

bool JWTValidator::is_expired(const std::string& token) {
    auto first_dot = token.find('.');
    auto second_dot = token.find('.', first_dot + 1);
    
    if (first_dot == std::string::npos || second_dot == std::string::npos) {
        return true;
    }
    
    std::string payload_b64 = token.substr(first_dot + 1, second_dot - first_dot - 1);
    std::string payload_json = base64url_decode(payload_b64);
    
    // 简单提取 exp 字段
    std::regex exp_regex(R"("exp"\s*:\s*(\d+))");
    std::smatch match;
    
    if (std::regex_search(payload_json, match, exp_regex)) {
        int64_t exp = std::stoll(match[1].str());
        auto now = std::chrono::system_clock::now();
        auto now_sec = std::chrono::duration_cast<std::chrono::seconds>(
            now.time_since_epoch()).count();
        
        return now_sec > exp;
    }
    
    return true;  // 没有 exp 字段，视为过期
}

std::string JWTValidator::base64url_decode(const std::string& input) {
    // Base64URL 转 Base64
    std::string base64 = input;
    std::replace(base64.begin(), base64.end(), '-', '+');
    std::replace(base64.begin(), base64.end(), '_', '/');
    
    // 补齐 padding
    while (base64.size() % 4 != 0) {
        base64 += '=';
    }
    
    // Base64 解码
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
        return false;  // 暂只支持 HS256
    }
    
    // 计算 HMAC-SHA256
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int digest_len = 0;
    
    HMAC(EVP_sha256(),
         secret_.c_str(), static_cast<int>(secret_.size()),
         reinterpret_cast<const unsigned char*>(header_payload.c_str()),
         header_payload.size(),
         digest, &digest_len);
    
    // Base64URL 编码
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
    
    // 移除 padding（Base64URL 不需要）
    while (!computed.empty() && computed.back() == '=') {
        computed.pop_back();
    }
    
    return computed == signature;
}

std::optional<UserInfo> JWTValidator::parse_payload(const std::string& payload_json) {
    UserInfo info;
    
    // 简单正则提取（实际项目建议使用 JSON 库）
    std::regex user_id_regex(R"("user_id"\s*:\s*(\d+))");
    std::regex username_regex(R"("username"\s*:\s*"([^"]+)")");
    std::regex role_regex(R"("role"\s*:\s*"([^"]+)")");
    
    std::smatch match;
    
    if (std::regex_search(payload_json, match, user_id_regex)) {
        info.user_id = std::stoll(match[1].str());
    }
    
    if (std::regex_search(payload_json, match, username_regex)) {
        info.username = match[1].str();
    }
    
    if (std::regex_search(payload_json, match, role_regex)) {
        info.role = match[1].str();
    }
    
    // 简单权限解析
    std::regex perm_regex(R"("permissions"\s*:\s*\[([^\]]*)\])");
    if (std::regex_search(payload_json, match, perm_regex)) {
        std::string perms = match[1].str();
        std::regex single_perm(R"("([^"]+)")");
        auto perms_begin = std::sregex_iterator(perms.begin(), perms.end(), single_perm);
        auto perms_end = std::sregex_iterator();
        
        for (auto it = perms_begin; it != perms_end; ++it) {
            info.permissions.push_back((*it)[1].str());
        }
    }
    
    // 至少要有 user_id
    if (info.user_id == 0) {
        return std::nullopt;
    }
    
    return info;
}

} // namespace qyh::dataplane
