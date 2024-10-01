#pragma once

#include <map>
#include <set>
#include <vector>

#define ROS2

#ifdef INTER_ROBOT_COMMS_ESP32
#include "esp_log.h"
#elif defined(ROS2)
#include "rclcpp/logging.hpp"
#define ESP_LOGE(tag, format, ...) RCLCPP_ERROR(rclcpp::get_logger(tag), format, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) RCLCPP_WARN(rclcpp::get_logger(tag), format, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) RCLCPP_INFO(rclcpp::get_logger(tag), format, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...) RCLCPP_DEBUG(rclcpp::get_logger(tag), format, ##__VA_ARGS__)
#define ESP_LOGV(tag, format, ...) RCLCPP_DEBUG(rclcpp::get_logger(tag), format, ##__VA_ARGS__)
#else
#include <stdarg.h>
#include <stdio.h>
inline void log_write(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    vprintf(format, arg);
    va_end(arg);
}

#define ESP_LOGE(tag, format, ...) log_write("[%s - ERROR]: " format, tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) log_write("[%s - WARN]: " format, tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) log_write("[%s - INFO]: " format, tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...) log_write("[%s - DEBUG]: " format, tag, ##__VA_ARGS__)
#define ESP_LOGV(tag, format, ...) log_write("[%s - VERBOSE]: " format, tag, ##__VA_ARGS__)
#endif

#include <stdint.h>
#include <string>

#define MAX_ROBOTS 10
#define MAX_HOST_LEN 18

template <class T, std::size_t Size> struct static_allocator {
    using value_type = T;
#if __cplusplus < 202002L
    template <class U> struct rebind {
        using other = static_allocator<U, Size>;
    };
#endif

    static_allocator() = default;
    template <class U, std::size_t Size2>
    constexpr explicit static_allocator(const static_allocator<U, Size2> &) noexcept
    {
    }

    alignas(T) static std::array<uint8_t, (Size) * sizeof(T)> buffer_;
    static std::size_t offset_;
    static std::array<uint8_t, 1 + ((Size) / 8)> mask;

    static auto allocate() -> T *
    { /* Find the first bit in the mask that is 0. */
        std::size_t idx_byte = 0;
        while (idx_byte <= ((Size) / 8) && mask.at(idx_byte) == 0xFF)
        {
            idx_byte++;
        }
        if (idx_byte > ((Size) / 8)) { throw std::bad_alloc{}; }

        /* Find the first bit in the byte that is 0. */
        std::size_t idx_bit = 0;
        while (mask.at(idx_byte) & (1U << idx_bit))
        {
            idx_bit++;
        }

        /* Set the bit to 1. */
        mask.at(idx_byte) |= (1U << idx_bit);

        /* Calculate the offset. */
        const auto new_offset = (idx_byte * 8) + idx_bit;
        auto place            = buffer_.data() + (new_offset * sizeof(T));
        offset_               = new_offset;

        return static_cast<T *>(static_cast<void *>(place));
    }

    static auto allocate(std::size_t n) -> T *
    {
        T *ptr = allocate();
        for (std::size_t idx = 1; idx < n; idx++)
        {
            allocate();
        }
        return ptr;
    }

    static void deallocate(T *ptr) noexcept
    { /* Calculate the offset. */
        const auto new_offset = (static_cast<uint8_t *>(static_cast<void *>(ptr)) - buffer_.data()) / sizeof(T);

        /* Find the byte and bit. */
        const std::size_t idx_byte = new_offset / 8U;
        const std::size_t idx_bit  = new_offset % 8U;

        /* Set the bit to 0. */
        mask.at(idx_byte) &= ~(1U << idx_bit);
    }

    static void deallocate(T *ptr, std::size_t n) noexcept
    {
        for (std::size_t idx = 0; idx < n; idx++)
        {
            deallocate(ptr + idx);
        }
    }
};

template <class T, class U, std::size_t Size1, std::size_t Size2>
constexpr bool operator==(const static_allocator<T, Size1> &, const static_allocator<U, Size2> &) noexcept
{
    return true;
}

template <class T, class U, std::size_t Size1, std::size_t Size2>
constexpr bool operator!=(const static_allocator<T, Size1> &, const static_allocator<U, Size2> &) noexcept
{
    return false;
}

template <class T, std::size_t Size> std::size_t static_allocator<T, Size>::offset_ = 0;
template <class T, std::size_t Size>
alignas(T) std::array<uint8_t, (Size) * sizeof(T)> static_allocator<T, Size>::buffer_                      = {0};
template <class T, std::size_t Size> std::array<uint8_t, 1 + ((Size) / 8)> static_allocator<T, Size>::mask = {0};

/* Common allocators and container types */
template <class T> using RobotSizeAllocator = static_allocator<T, MAX_ROBOTS>;
template <class T> using HostSizeAllocator  = static_allocator<T, MAX_HOST_LEN>;

template <class T> using RobotSizeVector = std::vector<T, RobotSizeAllocator<T>>;
template <class T> using RobotSizeSet    = std::set<T, std::less<T>, RobotSizeAllocator<T>>;
template <class T, class U> using RobotSizeMap =
    std::map<const T, U, std::less<T>, RobotSizeAllocator<std::pair<const T, U>>>;

using HostSizeString = std::basic_string<char, std::char_traits<char>, HostSizeAllocator<char>>;
