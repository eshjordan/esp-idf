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

struct PACKED EpuckHeartbeatPacket {
    uint8_t id                                              = 0x20;
    robot_id_type robot_id                                  = 0;
    std::array<char, MAX_HOST_LEN + 1> robot_comms_host     = {0};
    uint16_t robot_comms_request_port                       = 0;
    std::array<char, MAX_HOST_LEN + 1> robot_knowledge_host = {0};
    uint16_t robot_knowledge_exchange_port                  = 0;

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckHeartbeatPacket)> buffer = {0};

        auto *id_ptr       = static_cast<uint8_t *>(&buffer[offsetof(EpuckHeartbeatPacket, id)]);
        auto *robot_id_ptr = reinterpret_cast<robot_id_type *>(&buffer[offsetof(EpuckHeartbeatPacket, robot_id)]);
        auto *robot_comms_host_ptr =
            reinterpret_cast<char *>(&buffer[offsetof(EpuckHeartbeatPacket, robot_comms_host)]);
        auto *robot_comms_request_port_ptr =
            reinterpret_cast<uint16_t *>(&buffer[offsetof(EpuckHeartbeatPacket, robot_comms_request_port)]);
        auto *robot_knowledge_host_ptr =
            reinterpret_cast<char *>(&buffer[offsetof(EpuckHeartbeatPacket, robot_knowledge_host)]);
        auto *robot_knowledge_exchange_port_ptr =
            reinterpret_cast<uint16_t *>(&buffer[offsetof(EpuckHeartbeatPacket, robot_knowledge_exchange_port)]);

        impl::value_to_buffer(id_ptr, id);
        impl::value_to_buffer(robot_id_ptr, robot_id);
        strncpy(robot_comms_host_ptr, robot_comms_host.data(), MAX_HOST_LEN);
        impl::value_to_buffer(robot_comms_request_port_ptr, robot_comms_request_port);
        strncpy(robot_knowledge_host_ptr, robot_knowledge_host.data(), MAX_HOST_LEN);
        impl::value_to_buffer(robot_knowledge_exchange_port_ptr, robot_knowledge_exchange_port);

        return buffer;
    }

    [[nodiscard]] static EpuckHeartbeatPacket unpack(const void *const buffer)
    {
        EpuckHeartbeatPacket packet;

        const auto *id_ptr       = &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatPacket, id)];
        const auto *robot_id_ptr = reinterpret_cast<const robot_id_type *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatPacket, robot_id)]);
        const auto *robot_comms_host_ptr = reinterpret_cast<const char *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatPacket, robot_comms_host)]);
        const auto *robot_comms_request_port_ptr = reinterpret_cast<const uint16_t *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatPacket, robot_comms_request_port)]);
        const auto *robot_knowledge_host_ptr = reinterpret_cast<const char *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatPacket, robot_knowledge_host)]);
        const auto *robot_knowledge_exchange_port_ptr = reinterpret_cast<const uint16_t *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckHeartbeatPacket, robot_knowledge_exchange_port)]);

        packet.id       = impl::buffer_to_value<decltype(packet.id)>(id_ptr);
        packet.robot_id = impl::buffer_to_value<decltype(packet.robot_id)>(robot_id_ptr);
        strncpy(packet.robot_comms_host.data(), robot_comms_host_ptr, MAX_HOST_LEN);
        packet.robot_comms_request_port =
            impl::buffer_to_value<decltype(packet.robot_comms_request_port)>(robot_comms_request_port_ptr);
        strncpy(packet.robot_knowledge_host.data(), robot_knowledge_host_ptr, MAX_HOST_LEN);
        packet.robot_knowledge_exchange_port =
            impl::buffer_to_value<decltype(packet.robot_knowledge_exchange_port)>(robot_knowledge_exchange_port_ptr);

        return packet;
    }
};

struct PACKED EpuckNeighbourPacket {
    robot_id_type robot_id;
    std::array<char, MAX_HOST_LEN + 1> host;
    uint16_t port;
    float dist;

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

struct PACKED EpuckHeartbeatResponsePacket {
    uint8_t id                                              = 0x21;
    uint8_t num_neighbours                                  = 0;
    std::array<EpuckNeighbourPacket, MAX_ROBOTS> neighbours = {};

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
        static_assert(std::is_pod_v<EpuckNeighbourPacket>);
        static_assert(std::is_standard_layout_v<EpuckNeighbourPacket>);
        static_assert(std::is_trivial<EpuckNeighbourPacket>::value);

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

struct PACKED Centroid {
    float x;
    float y;
    float z;

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(Centroid)> buffer = {0};

        auto *x_ptr = reinterpret_cast<uint32_t *>(&buffer[offsetof(Centroid, x)]);
        auto *y_ptr = reinterpret_cast<uint32_t *>(&buffer[offsetof(Centroid, y)]);
        auto *z_ptr = reinterpret_cast<uint32_t *>(&buffer[offsetof(Centroid, z)]);

        impl::value_to_buffer(x_ptr, x);
        impl::value_to_buffer(y_ptr, y);
        impl::value_to_buffer(z_ptr, z);

        return buffer;
    }

    [[nodiscard]] static Centroid unpack(const void *const buffer)
    {
        Centroid centroid;

        const auto *x_ptr =
            reinterpret_cast<const uint32_t *>(&static_cast<const uint8_t *>(buffer)[offsetof(Centroid, x)]);
        const auto *y_ptr =
            reinterpret_cast<const uint32_t *>(&static_cast<const uint8_t *>(buffer)[offsetof(Centroid, y)]);
        const auto *z_ptr =
            reinterpret_cast<const uint32_t *>(&static_cast<const uint8_t *>(buffer)[offsetof(Centroid, z)]);

        centroid.x = impl::buffer_to_value<decltype(centroid.x)>(x_ptr);
        centroid.y = impl::buffer_to_value<decltype(centroid.y)>(y_ptr);
        centroid.z = impl::buffer_to_value<decltype(centroid.z)>(z_ptr);

        return centroid;
    }
};

struct PACKED Boundary {
    std::array<float, MAX_BOUNDARY_X_POINTS> x_points;
    std::array<float, MAX_BOUNDARY_Y_POINTS> y_points;
    std::array<float, MAX_BOUNDARY_Z_POINTS> z_points;

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(Boundary)> buffer = {0};

        auto *x_points_ptr = reinterpret_cast<uint32_t *>(&buffer[offsetof(Boundary, x_points)]);
        auto *y_points_ptr = reinterpret_cast<uint32_t *>(&buffer[offsetof(Boundary, y_points)]);
        auto *z_points_ptr = reinterpret_cast<uint32_t *>(&buffer[offsetof(Boundary, z_points)]);

        for (int i = 0; i < MAX_BOUNDARY_X_POINTS; i++)
        {
            impl::value_to_buffer(&x_points_ptr[i], x_points.at(i));
        }

        for (int i = 0; i < MAX_BOUNDARY_Y_POINTS; i++)
        {
            impl::value_to_buffer(&y_points_ptr[i], y_points.at(i));
        }

        for (int i = 0; i < MAX_BOUNDARY_Z_POINTS; i++)
        {
            impl::value_to_buffer(&z_points_ptr[i], z_points.at(i));
        }

        return buffer;
    }

    [[nodiscard]] static Boundary unpack(const void *const buffer)
    {
        Boundary boundary;

        const auto *x_points_ptr =
            reinterpret_cast<const uint32_t *>(&static_cast<const uint8_t *>(buffer)[offsetof(Boundary, x_points)]);
        const auto *y_points_ptr =
            reinterpret_cast<const uint32_t *>(&static_cast<const uint8_t *>(buffer)[offsetof(Boundary, y_points)]);
        const auto *z_points_ptr =
            reinterpret_cast<const uint32_t *>(&static_cast<const uint8_t *>(buffer)[offsetof(Boundary, z_points)]);

        for (int i = 0; i < MAX_BOUNDARY_X_POINTS; i++)
        {
            boundary.x_points.at(i) =
                impl::buffer_to_value<std::remove_reference_t<decltype(boundary.x_points[0])>>(&x_points_ptr[i]);
        }

        for (int i = 0; i < MAX_BOUNDARY_Y_POINTS; i++)
        {
            boundary.y_points.at(i) =
                impl::buffer_to_value<std::remove_reference_t<decltype(boundary.y_points[0])>>(&y_points_ptr[i]);
        }

        for (int i = 0; i < MAX_BOUNDARY_Z_POINTS; i++)
        {
            boundary.z_points.at(i) =
                impl::buffer_to_value<std::remove_reference_t<decltype(boundary.z_points[0])>>(&z_points_ptr[i]);
        }

        return boundary;
    }
};

struct PACKED EpuckKnowledgeRecord {
    robot_id_type robot_id;
    Centroid centroid;
    Boundary boundary;
    uint16_t seq;

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckKnowledgeRecord)> buffer = {0};

        auto *robot_id_ptr = reinterpret_cast<robot_id_type *>(&buffer[offsetof(EpuckKnowledgeRecord, robot_id)]);
        auto *centroid_ptr = reinterpret_cast<Centroid *>(&buffer[offsetof(EpuckKnowledgeRecord, centroid)]);
        auto *boundary_ptr = reinterpret_cast<Boundary *>(&buffer[offsetof(EpuckKnowledgeRecord, boundary)]);
        auto *seq_ptr      = reinterpret_cast<uint16_t *>(&buffer[offsetof(EpuckKnowledgeRecord, seq)]);

        impl::value_to_buffer(robot_id_ptr, robot_id);
        memcpy(centroid_ptr, centroid.pack().data(), sizeof(centroid));
        memcpy(boundary_ptr, boundary.pack().data(), sizeof(boundary));
        impl::value_to_buffer(seq_ptr, seq);

        return buffer;
    }

    [[nodiscard]] static EpuckKnowledgeRecord unpack(const void *const buffer)
    {
        EpuckKnowledgeRecord record;

        const auto *robot_id_ptr = reinterpret_cast<const robot_id_type *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgeRecord, robot_id)]);
        const auto *centroid_ptr = reinterpret_cast<const Centroid *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgeRecord, centroid)]);
        const auto *boundary_ptr = reinterpret_cast<const Boundary *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgeRecord, boundary)]);
        const auto *seq_ptr = reinterpret_cast<const uint16_t *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgeRecord, seq)]);

        record.robot_id = impl::buffer_to_value<decltype(record.robot_id)>(robot_id_ptr);
        record.centroid = Centroid::unpack(centroid_ptr);
        record.boundary = Boundary::unpack(boundary_ptr);
        record.seq      = impl::buffer_to_value<decltype(record.seq)>(seq_ptr);

        return record;
    }

    bool operator<(const EpuckKnowledgeRecord &other) const
    {
        return (robot_id < other.robot_id) || (robot_id == other.robot_id && seq < other.seq);
    }

    bool operator==(const EpuckKnowledgeRecord &other) const { return robot_id == other.robot_id && seq == other.seq; }
};

struct PACKED EpuckKnowledgePacket {
    uint8_t id                                             = 0x22;
    robot_id_type robot_id                                 = 0;
    uint16_t seq                                           = 0;
    uint8_t N                                              = 0;
    std::array<EpuckKnowledgeRecord, MAX_ROBOTS> known_ids = {};

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckKnowledgePacket)> buffer = {0};

        auto *id_ptr       = static_cast<uint8_t *>(&buffer[offsetof(EpuckKnowledgePacket, id)]);
        auto *robot_id_ptr = reinterpret_cast<robot_id_type *>(&buffer[offsetof(EpuckKnowledgePacket, robot_id)]);
        auto *seq_ptr      = reinterpret_cast<uint16_t *>(&buffer[offsetof(EpuckKnowledgePacket, seq)]);
        auto *N_ptr        = static_cast<uint8_t *>(&buffer[offsetof(EpuckKnowledgePacket, N)]);
        auto *known_ids_ptr =
            reinterpret_cast<EpuckKnowledgeRecord *>(&buffer[offsetof(EpuckKnowledgePacket, known_ids)]);

        impl::value_to_buffer(id_ptr, id);
        impl::value_to_buffer(robot_id_ptr, robot_id);
        impl::value_to_buffer(seq_ptr, seq);
        impl::value_to_buffer(N_ptr, N);

        for (int i = 0; i < N; i++)
        {
            memcpy(&known_ids_ptr[i], known_ids.at(i).pack().data(), sizeof(known_ids[0]));
        }

        return buffer;
    }

    [[nodiscard]] static EpuckKnowledgePacket unpack(const void *const buffer)
    {
        EpuckKnowledgePacket packet;

        const auto *id_ptr       = &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgePacket, id)];
        const auto *robot_id_ptr = reinterpret_cast<const robot_id_type *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgePacket, robot_id)]);
        const auto *seq_ptr = reinterpret_cast<const uint16_t *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgePacket, seq)]);
        const auto *N_ptr         = &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgePacket, N)];
        const auto *known_ids_ptr = reinterpret_cast<const EpuckKnowledgeRecord *>(
            &static_cast<const uint8_t *>(buffer)[offsetof(EpuckKnowledgePacket, known_ids)]);

        packet.id       = impl::buffer_to_value<decltype(packet.id)>(id_ptr);
        packet.robot_id = impl::buffer_to_value<decltype(packet.robot_id)>(robot_id_ptr);
        packet.seq      = impl::buffer_to_value<decltype(packet.seq)>(seq_ptr);
        packet.N        = impl::buffer_to_value<decltype(packet.N)>(N_ptr);

        for (int i = 0; i < packet.N; i++)
        {
            packet.known_ids.at(i) = EpuckKnowledgeRecord::unpack(&known_ids_ptr[i]);
        }

        return packet;
    }
};
