/**
 * @file auth.cpp
 * @brief JWT 认证实现
 */

#include "media_plane/auth.hpp"

#include <nlohmann/json.hpp>
#include <sstream>
#include <algorithm>
#include <ctime>
#include <iostream>
#include <vector>
#include <openssl/hmac.h>
#include <openssl/evp.h>
#include <openssl/crypto.h>

using json = nlohmann::json;

namespace qyh::mediaplane {

// ==================== JwtVerifier ====================

JwtVerifier::JwtVerifier(const std::string& secret)
    : secret_(secret)
{
}

std::string JwtVerifier::base64_url_decode(const std::string& input) {
    std::string output = input;
    
    // 替换 URL 安全字符
    std::replace(output.begin(), output.end(), '-', '+');
    std::replace(output.begin(), output.end(), '_', '/');
    
    // 添加填充
    while (output.size() % 4 != 0) {
        output += '=';
    }
    
    // Base64 解码
    static const std::string base64_chars = 
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    
    std::string decoded;
    std::vector<int> T(256, -1);
    for (int i = 0; i < 64; i++) {
        T[base64_chars[i]] = i;
    }
    
    int val = 0, valb = -8;
    for (unsigned char c : output) {
        if (T[c] == -1) break;
        val = (val << 6) + T[c];
        valb += 6;
        if (valb >= 0) {
            decoded.push_back(char((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    
    return decoded;
}

std::optional<UserInfo> JwtVerifier::verify(const std::string& token) const {
    // 分割 Token
    std::vector<std::string> parts;
    std::istringstream ss(token);
    std::string part;
    while (std::getline(ss, part, '.')) {
        parts.push_back(part);
    }
    
    if (parts.size() != 3) {
        return std::nullopt;
    }
    
    // 解析 Header，校验 alg
    std::string header_json = base64_url_decode(parts[0]);
    try {
        auto header = json::parse(header_json);
        if (header.contains("alg")) {
            auto alg = header["alg"].get<std::string>();
            if (alg != "HS256") {
                return std::nullopt;
            }
        }
    } catch (const json::exception&) {
        return std::nullopt;
    }

    // 验证签名
    std::string header_payload = parts[0] + "." + parts[1];
    if (!verify_signature(header_payload, parts[2])) {
        return std::nullopt;
    }
    
    // 解码 Payload
    std::string payload_json = base64_url_decode(parts[1]);
    
    try {
        auto payload = json::parse(payload_json);
        
        UserInfo info;
        
        // 提取字段
        if (payload.contains("sub")) {
            info.user_id = payload["sub"].get<std::string>();
        }
        if (payload.contains("username")) {
            info.username = payload["username"].get<std::string>();
        }
        if (payload.contains("role")) {
            info.role = payload["role"].get<std::string>();
        }
        
        // 检查过期时间
        if (payload.contains("exp")) {
            auto exp = payload["exp"].get<int64_t>();
            info.expires_at = std::chrono::system_clock::from_time_t(exp);
            
            if (info.is_expired()) {
                return std::nullopt;
            }
        }
        
        return info;
        
    } catch (const json::exception& e) {
        std::cerr << "JWT parse error: " << e.what() << std::endl;
        return std::nullopt;
    }
}

bool JwtVerifier::verify_signature(const std::string& header_payload,
                                    const std::string& signature) const {
    if (secret_.empty()) {
        return false;
    }

    // 解码签名（Base64 URL）
    std::string sig_bytes = base64_url_decode(signature);
    if (sig_bytes.empty()) {
        return false;
    }

    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int digest_len = 0;

    const auto* key = reinterpret_cast<const unsigned char*>(secret_.data());
    const auto* data = reinterpret_cast<const unsigned char*>(header_payload.data());

    if (!HMAC(EVP_sha256(), key, static_cast<int>(secret_.size()),
              data, static_cast<int>(header_payload.size()), digest, &digest_len)) {
        return false;
    }

    if (sig_bytes.size() != digest_len) {
        return false;
    }

    // 常量时间比较
    return CRYPTO_memcmp(sig_bytes.data(), digest, digest_len) == 0;
}

// ==================== SimpleJwtVerifier ====================

SimpleJwtVerifier::SimpleJwtVerifier(const std::string& secret)
    : secret_(secret)
{
}

std::optional<UserInfo> SimpleJwtVerifier::verify(const std::string& token) const {
    // 使用 JwtVerifier 的验证逻辑
    JwtVerifier verifier(secret_);
    return verifier.verify(token);
}

} // namespace qyh::mediaplane
