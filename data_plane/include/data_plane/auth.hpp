/**
 * @file auth.hpp
 * @brief JWT 认证模块
 */

#pragma once

#include <string>
#include <optional>
#include "session.hpp"

namespace qyh::dataplane {

/**
 * @brief JWT 验证器
 */
class JWTValidator {
public:
    /**
     * @brief 构造函数
     * @param secret JWT 密钥
     * @param algorithm 算法（默认 HS256）
     */
    explicit JWTValidator(const std::string& secret, 
                          const std::string& algorithm = "HS256");
    
    /**
     * @brief 验证 JWT Token
     * @param token JWT Token 字符串
     * @return 用户信息，验证失败返回 std::nullopt
     */
    std::optional<UserInfo> validate(const std::string& token);
    
    /**
     * @brief 验证 Token 是否过期
     * @param token JWT Token 字符串
     * @return true 表示已过期或无效
     */
    bool is_expired(const std::string& token);
    
private:
    /**
     * @brief Base64URL 解码
     */
    std::string base64url_decode(const std::string& input);
    
    /**
     * @brief 验证签名
     */
    bool verify_signature(const std::string& header_payload, 
                          const std::string& signature);
    
    /**
     * @brief 解析 JSON payload
     */
    std::optional<UserInfo> parse_payload(const std::string& payload_json);
    
private:
    std::string secret_;
    std::string algorithm_;
};

} // namespace qyh::dataplane
