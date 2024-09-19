#pragma once

#ifdef __cplusplus
extern "C++" {
#endif

#include <string>

#define MAX_ROBOTS 10
#define MAX_HOST_LEN 18

#define DECLARE_STATIC_ALLOCATOR(AllocatorName, Size)                                                                  \
    template <typename T> struct AllocatorName {                                                                       \
        using value_type = T;                                                                                          \
        AllocatorName() noexcept {}                                                                                    \
        template <class U> AllocatorName(const AllocatorName<U> &) noexcept {}                                         \
                                                                                                                       \
        alignas(T) static uint8_t buffer_[(Size) * sizeof(T)];                                                         \
        static std::size_t offset_;                                                                                    \
        static uint8_t mask[1 + (Size) / 8];                                                                           \
                                                                                                                       \
        static auto allocate() -> T *                                                                                  \
        {                                                                                                              \
            /* Find the first bit in the mask that is 0. */                                                            \
            std::size_t i = 0;                                                                                         \
            while (i <= (Size) / 8 && mask[i] == 0xFF)                                                                 \
            {                                                                                                          \
                i++;                                                                                                   \
            }                                                                                                          \
            if (i > (Size) / 8) throw std::bad_alloc{};                                                                \
                                                                                                                       \
            /* Find the first bit in the byte that is 0. */                                                            \
            std::size_t j = 0;                                                                                         \
            while (mask[i] & (1 << j))                                                                                 \
            {                                                                                                          \
                j++;                                                                                                   \
            }                                                                                                          \
                                                                                                                       \
            /* Set the bit to 1. */                                                                                    \
            mask[i] |= (1 << j);                                                                                       \
                                                                                                                       \
            /* Calculate the offset. */                                                                                \
            const auto new_offset = i * 8 + j;                                                                         \
            auto place            = reinterpret_cast<T *>(buffer_ + new_offset * sizeof(T));                           \
            offset_               = new_offset;                                                                        \
                                                                                                                       \
            return place;                                                                                              \
        }                                                                                                              \
                                                                                                                       \
        static auto allocate(std::size_t n) -> T *                                                                     \
        {                                                                                                              \
            T *ptr = allocate();                                                                                       \
            for (std::size_t i = 1; i < n; i++)                                                                        \
            {                                                                                                          \
                allocate();                                                                                            \
            }                                                                                                          \
            return ptr;                                                                                                \
        }                                                                                                              \
                                                                                                                       \
        static void deallocate(T *ptr) noexcept                                                                        \
        {                                                                                                              \
            /* Calculate the offset. */                                                                                \
            const auto place      = reinterpret_cast<uint8_t *>(ptr);                                                  \
            const auto new_offset = (place - buffer_) / sizeof(T);                                                     \
                                                                                                                       \
            /* Find the byte and bit. */                                                                               \
            const std::size_t i = new_offset / 8;                                                                      \
            const std::size_t j = new_offset % 8;                                                                      \
                                                                                                                       \
            /* Set the bit to 0. */                                                                                    \
            mask[i] &= ~(1 << j);                                                                                      \
        }                                                                                                              \
                                                                                                                       \
        static void deallocate(T *ptr, std::size_t n) noexcept                                                         \
        {                                                                                                              \
            for (std::size_t i = 0; i < n; i++)                                                                        \
            {                                                                                                          \
                deallocate(ptr + i);                                                                                   \
            }                                                                                                          \
        }                                                                                                              \
    };                                                                                                                 \
                                                                                                                       \
    template <class T, class U> constexpr bool operator==(const AllocatorName<T> &, const AllocatorName<U> &) noexcept \
    {                                                                                                                  \
        return true;                                                                                                   \
    }                                                                                                                  \
                                                                                                                       \
    template <class T, class U> constexpr bool operator!=(const AllocatorName<T> &, const AllocatorName<U> &) noexcept \
    {                                                                                                                  \
        return false;                                                                                                  \
    }                                                                                                                  \
                                                                                                                       \
    template <typename T> alignas(T) uint8_t AllocatorName<T>::buffer_[(Size) * sizeof(T)] = {};                       \
    template <typename T> std::size_t AllocatorName<T>::offset_                            = 0;                        \
    template <typename T> uint8_t AllocatorName<T>::mask[1 + (Size) / 8]                   = {};

DECLARE_STATIC_ALLOCATOR(RobotSizeAllocator, MAX_ROBOTS)

DECLARE_STATIC_ALLOCATOR(HostSizeAllocator, MAX_HOST_LEN)

using HostString = std::basic_string<char, std::char_traits<char>, RobotSizeAllocator<char>>;

#ifdef __cplusplus
}
#endif
