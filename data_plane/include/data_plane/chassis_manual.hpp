/**
 * @file chassis_manual.hpp
 * @brief 底盘手动控制模块
 * 
 * 提供精细的底盘手动控制功能：
 * - 摇杆输入处理（游戏手柄/VR 控制器）
 * - 键盘输入处理
 * - 死区处理
 * - 指数曲线（低速更精细）
 * - 速度平滑（加减速曲线）
 */

#pragma once

#include <cmath>
#include <chrono>
#include <mutex>
#include <functional>
#include <algorithm>

namespace qyh::dataplane {

// ==================== 配置 ====================

/**
 * @brief 手动控制配置
 */
struct ManualControlConfig {
    // 速度限制
    double max_linear_speed = 0.5;      // 最大线速度 (m/s)
    double max_angular_speed = 0.8;     // 最大角速度 (rad/s)
    
    // 平滑参数
    double acceleration = 0.3;          // 加速度 (m/s²)
    double deceleration = 0.5;          // 减速度 (m/s²)
    
    // 摇杆参数
    double dead_zone = 0.1;             // 死区 (0.0 ~ 0.3)
    double exponent = 2.0;              // 指数曲线系数
    bool use_exponential_curve = true;  // 是否使用指数曲线
    
    // 安全参数
    double command_timeout_ms = 200;    // 命令超时 (ms)
    double strafe_ratio = 0.5;          // 平移速度比例
};

// ==================== 输入类型 ====================

/**
 * @brief 摇杆输入（归一化 -1.0 ~ 1.0）
 */
struct JoystickInput {
    double x = 0;           // 左右 (-1=左, 1=右)
    double y = 0;           // 前后 (-1=后, 1=前)  
    double rotation = 0;    // 旋转 (-1=左转, 1=右转)
    double speed_multiplier = 1.0;  // 速度倍率 (0.1 ~ 1.0)
    
    bool is_zero() const {
        return std::abs(x) < 0.001 && std::abs(y) < 0.001 && std::abs(rotation) < 0.001;
    }
};

/**
 * @brief 键盘输入
 */
struct KeyboardInput {
    bool forward = false;
    bool backward = false;
    bool left = false;
    bool right = false;
    bool rotate_left = false;
    bool rotate_right = false;
    
    enum class SpeedLevel { SLOW, NORMAL, FAST };
    SpeedLevel speed_level = SpeedLevel::NORMAL;
    
    bool is_zero() const {
        return !forward && !backward && !left && !right && !rotate_left && !rotate_right;
    }
};

/**
 * @brief 速度输出
 */
struct VelocityOutput {
    double linear_x = 0;    // 前进速度 (m/s)
    double linear_y = 0;    // 平移速度 (m/s)
    double angular_z = 0;   // 角速度 (rad/s)
    
    bool is_zero() const {
        return std::abs(linear_x) < 0.001 && 
               std::abs(linear_y) < 0.001 && 
               std::abs(angular_z) < 0.001;
    }
};

// ==================== 处理器 ====================

/**
 * @brief 死区处理
 */
inline double apply_dead_zone(double value, double dead_zone) {
    if (std::abs(value) < dead_zone) {
        return 0.0;
    }
    // 重新映射，使死区外的值从 0 开始
    double sign = value > 0 ? 1.0 : -1.0;
    return sign * (std::abs(value) - dead_zone) / (1.0 - dead_zone);
}

/**
 * @brief 指数曲线（使低速更精细）
 */
inline double apply_exponential_curve(double value, double exponent = 2.0) {
    double sign = value >= 0 ? 1.0 : -1.0;
    return sign * std::pow(std::abs(value), exponent);
}

/**
 * @brief 限幅
 */
inline double clamp(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

/**
 * @brief 底盘手动控制器
 */
class ChassisManualController {
public:
    using VelocityCallback = std::function<void(const VelocityOutput&)>;
    
    explicit ChassisManualController(const ManualControlConfig& config = {})
        : config_(config)
        , last_update_(std::chrono::steady_clock::now())
    {}
    
    /**
     * @brief 设置配置
     */
    void set_config(const ManualControlConfig& config) {
        std::lock_guard<std::mutex> lock(mutex_);
        config_ = config;
    }
    
    /**
     * @brief 获取配置
     */
    ManualControlConfig get_config() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return config_;
    }
    
    /**
     * @brief 设置速度输出回调
     */
    void set_velocity_callback(VelocityCallback callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        velocity_callback_ = std::move(callback);
    }
    
    /**
     * @brief 处理摇杆输入
     */
    VelocityOutput process_joystick(const JoystickInput& input) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        last_update_ = std::chrono::steady_clock::now();
        
        // 应用死区
        double x = apply_dead_zone(input.x, config_.dead_zone);
        double y = apply_dead_zone(input.y, config_.dead_zone);
        double rotation = apply_dead_zone(input.rotation, config_.dead_zone);
        
        // 应用指数曲线
        if (config_.use_exponential_curve) {
            x = apply_exponential_curve(x, config_.exponent);
            y = apply_exponential_curve(y, config_.exponent);
            rotation = apply_exponential_curve(rotation, config_.exponent);
        }
        
        // 计算目标速度
        double multiplier = clamp(input.speed_multiplier, 0.1, 1.0);
        target_velocity_.linear_x = y * config_.max_linear_speed * multiplier;
        target_velocity_.linear_y = -x * config_.max_linear_speed * multiplier * config_.strafe_ratio;
        target_velocity_.angular_z = -rotation * config_.max_angular_speed * multiplier;
        
        // 应用平滑
        apply_smoothing();
        
        // 触发回调
        if (velocity_callback_) {
            velocity_callback_(current_velocity_);
        }
        
        return current_velocity_;
    }
    
    /**
     * @brief 处理键盘输入
     */
    VelocityOutput process_keyboard(const KeyboardInput& input) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        last_update_ = std::chrono::steady_clock::now();
        
        // 速度倍率
        double multiplier = 0.6;
        switch (input.speed_level) {
            case KeyboardInput::SpeedLevel::SLOW: multiplier = 0.3; break;
            case KeyboardInput::SpeedLevel::NORMAL: multiplier = 0.6; break;
            case KeyboardInput::SpeedLevel::FAST: multiplier = 1.0; break;
        }
        
        // 计算目标速度
        target_velocity_.linear_x = 0;
        target_velocity_.linear_y = 0;
        target_velocity_.angular_z = 0;
        
        // 前后
        if (input.forward) {
            target_velocity_.linear_x = config_.max_linear_speed * multiplier;
        } else if (input.backward) {
            target_velocity_.linear_x = -config_.max_linear_speed * multiplier;
        }
        
        // 左右平移
        if (input.left) {
            target_velocity_.linear_y = config_.max_linear_speed * multiplier * config_.strafe_ratio;
        } else if (input.right) {
            target_velocity_.linear_y = -config_.max_linear_speed * multiplier * config_.strafe_ratio;
        }
        
        // 旋转
        if (input.rotate_left) {
            target_velocity_.angular_z = config_.max_angular_speed * multiplier;
        } else if (input.rotate_right) {
            target_velocity_.angular_z = -config_.max_angular_speed * multiplier;
        }
        
        // 应用平滑
        apply_smoothing();
        
        // 触发回调
        if (velocity_callback_) {
            velocity_callback_(current_velocity_);
        }
        
        return current_velocity_;
    }
    
    /**
     * @brief 直接设置速度（不经过平滑）
     */
    VelocityOutput set_velocity_direct(double linear_x, double linear_y, double angular_z) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        last_update_ = std::chrono::steady_clock::now();
        
        current_velocity_.linear_x = clamp(linear_x, -config_.max_linear_speed, config_.max_linear_speed);
        current_velocity_.linear_y = clamp(linear_y, -config_.max_linear_speed, config_.max_linear_speed);
        current_velocity_.angular_z = clamp(angular_z, -config_.max_angular_speed, config_.max_angular_speed);
        target_velocity_ = current_velocity_;
        
        if (velocity_callback_) {
            velocity_callback_(current_velocity_);
        }
        
        return current_velocity_;
    }
    
    /**
     * @brief 紧急停止
     */
    void emergency_stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        target_velocity_ = VelocityOutput{};
        current_velocity_ = VelocityOutput{};
        
        if (velocity_callback_) {
            velocity_callback_(current_velocity_);
        }
    }
    
    /**
     * @brief 检查命令是否超时
     */
    bool is_timed_out() const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(now - last_update_).count();
        return elapsed > config_.command_timeout_ms;
    }
    
    /**
     * @brief 更新（周期调用，处理超时和平滑）
     */
    VelocityOutput update() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 检查超时
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double, std::milli>(now - last_update_).count();
        
        if (elapsed > config_.command_timeout_ms) {
            // 超时，目标速度置零
            target_velocity_ = VelocityOutput{};
        }
        
        // 应用平滑
        apply_smoothing();
        
        // 如果速度变化了，触发回调
        static VelocityOutput last_output;
        if (std::abs(current_velocity_.linear_x - last_output.linear_x) > 0.001 ||
            std::abs(current_velocity_.linear_y - last_output.linear_y) > 0.001 ||
            std::abs(current_velocity_.angular_z - last_output.angular_z) > 0.001) {
            
            if (velocity_callback_) {
                velocity_callback_(current_velocity_);
            }
            last_output = current_velocity_;
        }
        
        return current_velocity_;
    }
    
    /**
     * @brief 获取当前速度
     */
    VelocityOutput get_current_velocity() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_velocity_;
    }
    
    /**
     * @brief 获取目标速度
     */
    VelocityOutput get_target_velocity() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return target_velocity_;
    }
    
private:
    /**
     * @brief 应用速度平滑
     */
    void apply_smoothing() {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_smooth_time_).count();
        last_smooth_time_ = now;
        
        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;  // 默认 50Hz
        }
        
        // 分别对每个轴应用平滑
        current_velocity_.linear_x = smooth_value(
            current_velocity_.linear_x, target_velocity_.linear_x, dt);
        current_velocity_.linear_y = smooth_value(
            current_velocity_.linear_y, target_velocity_.linear_y, dt);
        current_velocity_.angular_z = smooth_value_angular(
            current_velocity_.angular_z, target_velocity_.angular_z, dt);
    }
    
    /**
     * @brief 平滑单个值（线速度）
     */
    double smooth_value(double current, double target, double dt) {
        double diff = target - current;
        double max_change;
        
        if (std::abs(target) > std::abs(current)) {
            // 加速
            max_change = config_.acceleration * dt;
        } else {
            // 减速
            max_change = config_.deceleration * dt;
        }
        
        if (std::abs(diff) <= max_change) {
            return target;
        }
        
        return current + (diff > 0 ? max_change : -max_change);
    }
    
    /**
     * @brief 平滑单个值（角速度）
     */
    double smooth_value_angular(double current, double target, double dt) {
        // 角速度使用更快的响应
        double diff = target - current;
        double max_change = config_.acceleration * 2.0 * dt;
        
        if (std::abs(diff) <= max_change) {
            return target;
        }
        
        return current + (diff > 0 ? max_change : -max_change);
    }
    
    ManualControlConfig config_;
    VelocityOutput current_velocity_;
    VelocityOutput target_velocity_;
    VelocityCallback velocity_callback_;
    
    std::chrono::steady_clock::time_point last_update_;
    std::chrono::steady_clock::time_point last_smooth_time_;
    
    mutable std::mutex mutex_;
};

// ==================== JSON 解析辅助 ====================

/**
 * @brief 从 JSON 解析摇杆输入
 * 
 * 期望格式:
 * {
 *   "x": 0.5,
 *   "y": 0.8,
 *   "rotation": 0.0,
 *   "speed_multiplier": 1.0
 * }
 */
inline JoystickInput parse_joystick_json(const std::string& json) {
    JoystickInput input;
    // 简单解析（生产环境应使用 nlohmann/json 或 rapidjson）
    auto get_double = [&json](const char* key, double default_val) -> double {
        std::string pattern = std::string("\"") + key + "\":";
        auto pos = json.find(pattern);
        if (pos == std::string::npos) return default_val;
        pos += pattern.length();
        while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) ++pos;
        return std::stod(json.substr(pos));
    };
    
    input.x = get_double("x", 0);
    input.y = get_double("y", 0);
    input.rotation = get_double("rotation", 0);
    input.speed_multiplier = get_double("speed_multiplier", 1.0);
    
    // 限幅
    input.x = clamp(input.x, -1.0, 1.0);
    input.y = clamp(input.y, -1.0, 1.0);
    input.rotation = clamp(input.rotation, -1.0, 1.0);
    input.speed_multiplier = clamp(input.speed_multiplier, 0.1, 1.0);
    
    return input;
}

/**
 * @brief 从 JSON 解析键盘输入
 * 
 * 期望格式:
 * {
 *   "forward": true,
 *   "backward": false,
 *   "left": false,
 *   "right": false,
 *   "rotate_left": false,
 *   "rotate_right": false,
 *   "speed_level": "normal"
 * }
 */
inline KeyboardInput parse_keyboard_json(const std::string& json) {
    KeyboardInput input;
    
    auto get_bool = [&json](const char* key) -> bool {
        std::string pattern = std::string("\"") + key + "\":";
        auto pos = json.find(pattern);
        if (pos == std::string::npos) return false;
        pos += pattern.length();
        while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) ++pos;
        return json.substr(pos, 4) == "true";
    };
    
    input.forward = get_bool("forward");
    input.backward = get_bool("backward");
    input.left = get_bool("left");
    input.right = get_bool("right");
    input.rotate_left = get_bool("rotate_left");
    input.rotate_right = get_bool("rotate_right");
    
    // 解析速度级别
    auto pos = json.find("\"speed_level\":");
    if (pos != std::string::npos) {
        if (json.find("\"slow\"", pos) != std::string::npos) {
            input.speed_level = KeyboardInput::SpeedLevel::SLOW;
        } else if (json.find("\"fast\"", pos) != std::string::npos) {
            input.speed_level = KeyboardInput::SpeedLevel::FAST;
        }
    }
    
    return input;
}

} // namespace qyh::dataplane
