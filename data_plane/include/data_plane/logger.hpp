/**
 * @file logger.hpp
 * @brief 简单的日志宏定义
 * 
 * 提供带时间戳的日志输出，确保在 systemd 环境下能正确显示
 */

#pragma once

#include <iostream>
#include <iomanip>
#include <chrono>
#include <sstream>

namespace qyh::dataplane {

/**
 * @brief 获取当前时间戳字符串
 * @return 格式: YYYY-MM-DD HH:MM:SS.mmm
 */
inline std::string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

} // namespace qyh::dataplane

// 日志宏定义（自动刷新缓冲区，避免 systemd 日志丢失）
#define LOG_INFO(msg) \
    do { \
        std::cout << "[" << qyh::dataplane::get_timestamp() << "] [INFO] " \
                  << msg << std::endl; \
    } while(0)

#define LOG_WARN(msg) \
    do { \
        std::cout << "[" << qyh::dataplane::get_timestamp() << "] [WARN] " \
                  << msg << std::endl; \
    } while(0)

#define LOG_ERROR(msg) \
    do { \
        std::cerr << "[" << qyh::dataplane::get_timestamp() << "] [ERROR] " \
                  << msg << std::endl; \
    } while(0)

#define LOG_DEBUG(msg) \
    do { \
        std::cout << "[" << qyh::dataplane::get_timestamp() << "] [DEBUG] " \
                  << msg << std::endl; \
    } while(0)
