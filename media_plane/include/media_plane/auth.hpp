/**
 * @file auth.hpp
 * @brief JWT 认证模块
 * 
 * 与 Control Plane 共享 JWT 密钥
 */

#pragma once

#include <string>
#include <chrono>
#include <optional>

namespace qyh::mediaplane {

/**
 * @brief 用户信息
 */
struct UserInfo {
    std::string user_id;
    std::string username;
    std::string role;           // admin, operator, viewer
    std::chrono::system_clock::time_point expires_at;
    
    bool is_expired() const {
        return std::chrono::system_clock::now() >= expires_at;
    }
    
    bool has_video_access() const {
        // 所有角色都可以查看视频
        return !is_expired();
    }
};

/**
 * @brief JWT 验证器
 */
class JwtVerifier {
public:
    /**
     * @brief 构造函数
     * @param secret JWT 密钥（与 Control Plane 共享）
     */
    explicit JwtVerifier(const std::string& secret);
    
    /**
     * @brief 验证 Token
     * @param token JWT Token
     * @return 用户信息（验证失败返回 nullopt）
     */
    std::optional<UserInfo> verify(const std::string& token) const;
    
    /**
     * @brief 设置密钥
     */
    void set_secret(const std::string& secret) { secret_ = secret; }
    
private:
    std::string secret_;
    
    /**
     * @brief Base64 URL 解码
     */
    static std::string base64_url_decode(const std::string& input);
    
    /**
     * @brief HMAC-SHA256 签名验证
     */
    bool verify_signature(const std::string& header_payload,
                          const std::string& signature) const;
};

/**
 * @brief 简单 JWT 验证（不依赖外部库）
 * 
 * 注意：这是简化版本，仅验证格式和过期时间
 * 生产环境应使用完整的 JWT 库
 */
class SimpleJwtVerifier {
public:
    explicit SimpleJwtVerifier(const std::string& secret);
    
    /**
     * @brief 验证 Token
     */
    std::optional<UserInfo> verify(const std::string& token) const;
    
private:
    std::string secret_;
};

} // namespace qyh::mediaplane
