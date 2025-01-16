#pragma once

#include "types.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <endian.h>
#include <string.h>
#include <type_traits>

#define PACKED __attribute__((packed, aligned(1)))

using robot_id_type = uint16_t;
#define ROBOT_ID_TYPE_FMT "%d"

namespace impl {

template <class T, class U, std::size_t N> using enable_if_both_types_are_size_n = std::enable_if_t<std::conjunction_v<
    std::is_same<std::integral_constant<std::size_t, sizeof(T)>, std::integral_constant<std::size_t, N>>,
    std::is_same<std::integral_constant<std::size_t, sizeof(U)>, std::integral_constant<std::size_t, N>>>>;

template <class T, class U, std::size_t N> using enable_if_both_types_are_size_n_return_t = std::enable_if_t<
    std::conjunction_v<
        std::is_same<std::integral_constant<std::size_t, sizeof(T)>, std::integral_constant<std::size_t, N>>,
        std::is_same<std::integral_constant<std::size_t, sizeof(U)>, std::integral_constant<std::size_t, N>>>,
    T>;

template <class T, class U> enable_if_both_types_are_size_n<T, U, 1> value_to_buffer(U *buffer, T value)
{
    memcpy(buffer, &value, 1);
}

template <class T, class U> enable_if_both_types_are_size_n_return_t<T, U, 1> buffer_to_value(const U *buffer)
{
    auto val_u8 = *reinterpret_cast<const uint8_t *>(buffer);
    T result;
    memcpy(&result, &val_u8, 1);
    return result;
}

template <class T, class U> enable_if_both_types_are_size_n<T, U, 2> value_to_buffer(U *buffer, T value)
{
    memcpy(buffer, &value, 2);
    *buffer = htole16(*buffer);
}

template <class T, class U> enable_if_both_types_are_size_n_return_t<T, U, 2> buffer_to_value(const U *buffer)
{
    uint16_t val_u16 = le16toh(*buffer);
    T result;
    memcpy(&result, &val_u16, 2);
    return result;
}

template <class T, class U> enable_if_both_types_are_size_n<T, U, 4> value_to_buffer(U *buffer, T value)
{
    memcpy(buffer, &value, 4);
    *buffer = htole32(*buffer);
}

template <class T, class U> enable_if_both_types_are_size_n_return_t<T, U, 4> buffer_to_value(const U *buffer)
{
    uint32_t val_u32 = le32toh(*buffer);
    T result;
    memcpy(&result, &val_u32, 4);
    return result;
}
} // namespace impl

class PACKED EpuckHeartbeatPacket
{
public:
    PACKED uint8_t id                                    = 0x20;
    PACKED robot_id_type robot_id                        = 0;
    PACKED std::array<char, MAX_HOST_LEN + 1> robot_host = {0};
    PACKED uint16_t robot_port                           = 0;

    EpuckHeartbeatPacket() = default;
    [[nodiscard]] explicit EpuckHeartbeatPacket(const void *const buffer) { *this = unpack(buffer); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckHeartbeatPacket)> buffer = {0};

        auto *id_ptr         = static_cast<uint8_t *>(&buffer[offsetof(EpuckHeartbeatPacket, id)]);
        auto *robot_id_ptr   = reinterpret_cast<robot_id_type *>(&buffer[offsetof(EpuckHeartbeatPacket, robot_id)]);
        auto *robot_host_ptr = reinterpret_cast<char *>(&buffer[offsetof(EpuckHeartbeatPacket, robot_host)]);
        auto *robot_port_ptr = reinterpret_cast<uint16_t *>(&buffer[offsetof(EpuckHeartbeatPacket, robot_port)]);

        impl::value_to_buffer(id_ptr, id);
        impl::value_to_buffer(robot_id_ptr, robot_id);
        strncpy(robot_host_ptr, robot_host.data(), MAX_HOST_LEN);
        impl::value_to_buffer(robot_port_ptr, robot_port);

        return buffer;
    }

    [[nodiscard]] static EpuckHeartbeatPacket unpack(const void *const buffer)
    {
        EpuckHeartbeatPacket packet;

        const auto *id_ptr       = &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatPacket, id)];
        const auto *robot_id_ptr = reinterpret_cast<const robot_id_type *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatPacket, robot_id)]);
        const auto *robot_host_ptr = reinterpret_cast<const char *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatPacket, robot_host)]);
        const auto *robot_port_ptr = reinterpret_cast<const uint16_t *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatPacket, robot_port)]);

        packet.id       = impl::buffer_to_value<decltype(packet.id)>(id_ptr);
        packet.robot_id = impl::buffer_to_value<decltype(packet.robot_id)>(robot_id_ptr);
        strncpy(packet.robot_host.data(), robot_host_ptr, MAX_HOST_LEN);
        packet.robot_port = impl::buffer_to_value<decltype(packet.robot_port)>(robot_port_ptr);

        return packet;
    }
};

class PACKED EpuckNeighbourPacket
{
public:
    PACKED robot_id_type robot_id                  = 0;
    PACKED std::array<char, MAX_HOST_LEN + 1> host = {0};
    PACKED uint16_t port                           = 0;
    PACKED float dist                              = 0;

    EpuckNeighbourPacket() = default;
    [[nodiscard]] explicit EpuckNeighbourPacket(const void *const buffer) { *this = unpack(buffer); }

    bool operator==(const EpuckNeighbourPacket &other) const
    {
        return robot_id == other.robot_id && host == other.host && port == other.port && dist == other.dist;
    }

    bool operator<(const EpuckNeighbourPacket &other) const { return robot_id < other.robot_id; }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckNeighbourPacket)> buffer = {0};

        auto *robot_id_ptr = reinterpret_cast<robot_id_type *>(&buffer[offsetof(EpuckNeighbourPacket, robot_id)]);
        auto *host_ptr     = reinterpret_cast<char *>(&buffer[offsetof(EpuckNeighbourPacket, host)]);
        auto *port_ptr     = reinterpret_cast<uint16_t *>(&buffer[offsetof(EpuckNeighbourPacket, port)]);
        auto *dist_ptr     = reinterpret_cast<uint32_t *>(&buffer[offsetof(EpuckNeighbourPacket, dist)]);

        impl::value_to_buffer(robot_id_ptr, robot_id);
        strncpy(host_ptr, host.data(), MAX_HOST_LEN);
        impl::value_to_buffer(port_ptr, port);
        impl::value_to_buffer(dist_ptr, dist);

        return buffer;
    }

    [[nodiscard]] static EpuckNeighbourPacket unpack(const void *const buffer)
    {
        EpuckNeighbourPacket packet;

        const auto *robot_id_ptr = reinterpret_cast<const robot_id_type *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckNeighbourPacket, robot_id)]);
        const auto *host_ptr =
            reinterpret_cast<const char *>(&static_cast<const uint8_t *>(buffer)[offsetof(EpuckNeighbourPacket, host)]);
        const auto *port_ptr = reinterpret_cast<const uint16_t *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckNeighbourPacket, port)]);
        const auto *dist_ptr = reinterpret_cast<const uint32_t *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckNeighbourPacket, dist)]);

        packet.robot_id = impl::buffer_to_value<decltype(packet.robot_id)>(robot_id_ptr);
        strncpy(packet.host.data(), host_ptr, MAX_HOST_LEN);
        packet.port = impl::buffer_to_value<decltype(packet.port)>(port_ptr);
        packet.dist = impl::buffer_to_value<decltype(packet.dist)>(dist_ptr);

        return packet;
    }
};

class PACKED EpuckHeartbeatResponsePacket
{
public:
    PACKED uint8_t id                                              = 0x21;
    PACKED uint8_t num_neighbours                                  = 0;
    PACKED std::array<EpuckNeighbourPacket, MAX_ROBOTS> neighbours = {};

    EpuckHeartbeatResponsePacket() = default;
    [[nodiscard]] explicit EpuckHeartbeatResponsePacket(const void *const buffer) { *this = unpack(buffer); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckHeartbeatResponsePacket)> buffer = {0};

        auto *id_ptr = static_cast<uint8_t *>(&buffer[offsetof(EpuckHeartbeatResponsePacket, id)]);
        auto *num_neighbours_ptr =
            static_cast<uint8_t *>(&buffer[offsetof(EpuckHeartbeatResponsePacket, num_neighbours)]);
        auto *neighbours_ptr =
            reinterpret_cast<EpuckNeighbourPacket *>(&buffer[offsetof(EpuckHeartbeatResponsePacket, neighbours)]);

        impl::value_to_buffer(id_ptr, id);
        impl::value_to_buffer(num_neighbours_ptr, num_neighbours);

        int i = 0;
        for (const auto &neighbour : neighbours)
        {
            memcpy(&neighbours_ptr[i], neighbour.pack().data(), sizeof(EpuckNeighbourPacket));
            i++;
        }

        return buffer;
    }

    [[nodiscard]] static EpuckHeartbeatResponsePacket unpack(const void *const buffer)
    {
        EpuckHeartbeatResponsePacket packet;

        const auto *id_ptr = &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatResponsePacket, id)];
        const auto *num_neighbours_ptr =
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatResponsePacket, num_neighbours)];
        const auto *neighbours_ptr = reinterpret_cast<const EpuckNeighbourPacket *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatResponsePacket, neighbours)]);

        packet.id             = impl::buffer_to_value<decltype(packet.id)>(id_ptr);
        packet.num_neighbours = impl::buffer_to_value<decltype(packet.num_neighbours)>(num_neighbours_ptr);

        for (int i = 0; i < packet.num_neighbours; i++)
        {
            packet.neighbours.at(i) = EpuckNeighbourPacket::unpack(&neighbours_ptr[i]);
        }

        return packet;
    }
};

class PACKED EpuckKnowledgePacket
{
public:
    PACKED uint8_t id                                      = 0x22;
    PACKED robot_id_type robot_id                          = 0;
    PACKED uint8_t N                                       = 0;
    PACKED std::array<robot_id_type, MAX_ROBOTS> known_ids = {0};

    EpuckKnowledgePacket() = default;
    [[nodiscard]] explicit EpuckKnowledgePacket(const void *const buffer) { *this = unpack(buffer); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckKnowledgePacket)> buffer = {0};

        auto *id_ptr        = static_cast<uint8_t *>(&buffer[offsetof(EpuckKnowledgePacket, id)]);
        auto *robot_id_ptr  = reinterpret_cast<robot_id_type *>(&buffer[offsetof(EpuckKnowledgePacket, robot_id)]);
        auto *N_ptr         = static_cast<uint8_t *>(&buffer[offsetof(EpuckKnowledgePacket, N)]);
        auto *known_ids_ptr = reinterpret_cast<robot_id_type *>(&buffer[offsetof(EpuckKnowledgePacket, known_ids)]);

        impl::value_to_buffer(id_ptr, id);
        impl::value_to_buffer(robot_id_ptr, robot_id);
        impl::value_to_buffer(N_ptr, N);

        for (int i = 0; i < N; i++)
        {
            impl::value_to_buffer(&known_ids_ptr[i], known_ids.at(i));
        }

        return buffer;
    }

    [[nodiscard]] static EpuckKnowledgePacket unpack(const void *const buffer)
    {
        EpuckKnowledgePacket packet;

        const auto *id_ptr       = &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgePacket, id)];
        const auto *robot_id_ptr = reinterpret_cast<const robot_id_type *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgePacket, robot_id)]);
        const auto *N_ptr         = &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgePacket, N)];
        const auto *known_ids_ptr = reinterpret_cast<const robot_id_type *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgePacket, known_ids)]);

        packet.id       = impl::buffer_to_value<decltype(packet.id)>(id_ptr);
        packet.robot_id = impl::buffer_to_value<decltype(packet.robot_id)>(robot_id_ptr);
        packet.N        = impl::buffer_to_value<decltype(packet.N)>(N_ptr);

        for (int i = 0; i < packet.N; i++)
        {
            packet.known_ids.at(i) =
                impl::buffer_to_value<std::remove_reference_t<decltype(packet.known_ids.at(i))>>(&known_ids_ptr[i]);
        }

        return packet;
    }
};

class PACKED EpuckAddressKnowledgePacket
{
public:
    PACKED uint8_t id                                      = 0x23;
    PACKED robot_id_type robot_id                          = 0;
    PACKED uint8_t N                                       = 0;
    PACKED std::array<char, MAX_HOST_LEN + 1> address      = {0};
    PACKED std::array<robot_id_type, MAX_ROBOTS> known_ids = {0};

    EpuckAddressKnowledgePacket() = default;
    [[nodiscard]] explicit EpuckAddressKnowledgePacket(const void *const buffer) { *this = unpack(buffer); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckAddressKnowledgePacket)> buffer = {0};

        auto *id_ptr = static_cast<uint8_t *>(&buffer[offsetof(EpuckAddressKnowledgePacket, id)]);
        auto *robot_id_ptr =
            reinterpret_cast<robot_id_type *>(&buffer[offsetof(EpuckAddressKnowledgePacket, robot_id)]);
        auto *N_ptr       = static_cast<uint8_t *>(&buffer[offsetof(EpuckAddressKnowledgePacket, N)]);
        auto *address_ptr = reinterpret_cast<char *>(&buffer[offsetof(EpuckAddressKnowledgePacket, address)]);
        auto *known_ids_ptr =
            reinterpret_cast<robot_id_type *>(&buffer[offsetof(EpuckAddressKnowledgePacket, known_ids)]);

        impl::value_to_buffer(id_ptr, id);
        impl::value_to_buffer(robot_id_ptr, robot_id);
        impl::value_to_buffer(N_ptr, N);
        strncpy(address_ptr, address.data(), MAX_HOST_LEN);

        for (int i = 0; i < N; i++)
        {
            impl::value_to_buffer(&known_ids_ptr[i], known_ids.at(i));
        }

        return buffer;
    }

    [[nodiscard]] static EpuckAddressKnowledgePacket unpack(const void *const buffer)
    {
        EpuckAddressKnowledgePacket packet;

        const auto *id_ptr       = &static_cast<const uint8_t *>(buffer)[offsetof(EpuckAddressKnowledgePacket, id)];
        const auto *robot_id_ptr = reinterpret_cast<const robot_id_type *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckAddressKnowledgePacket, robot_id)]);
        const auto *N_ptr       = &static_cast<const uint8_t *>(buffer)[offsetof(EpuckAddressKnowledgePacket, N)];
        const auto *address_ptr = reinterpret_cast<const char *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckAddressKnowledgePacket, address)]);
        const auto *known_ids_ptr = reinterpret_cast<const robot_id_type *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckAddressKnowledgePacket, known_ids)]);

        packet.id       = impl::buffer_to_value<decltype(packet.id)>(id_ptr);
        packet.robot_id = impl::buffer_to_value<decltype(packet.robot_id)>(robot_id_ptr);
        packet.N        = impl::buffer_to_value<decltype(packet.N)>(N_ptr);
        strncpy(packet.address.data(), address_ptr, MAX_HOST_LEN);

        for (int i = 0; i < packet.N; i++)
        {
            packet.known_ids.at(i) =
                impl::buffer_to_value<std::remove_reference_t<decltype(packet.known_ids.at(i))>>(&known_ids_ptr[i]);
        }

        return packet;
    }
};
