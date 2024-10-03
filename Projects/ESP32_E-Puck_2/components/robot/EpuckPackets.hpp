#pragma once

#include "types.hpp"
#include <array>
#include <cstdint>
#include <string.h>

#define PACKED __attribute__((packed, aligned(1)))

class PACKED EpuckHeartbeatPacket
{
public:
    PACKED uint8_t id                                    = 0x20;
    PACKED uint8_t robot_id                              = 0;
    PACKED std::array<char, MAX_HOST_LEN + 1> robot_host = {0};
    PACKED uint16_t robot_port                           = 0;

    EpuckHeartbeatPacket() = default;
    [[nodiscard]] explicit EpuckHeartbeatPacket(const void *const buffer) { *this = std::move(unpack(buffer)); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckHeartbeatPacket)> buffer = {0};
        buffer[0]                                                = id;
        buffer[1]                                                = robot_id;
        strncpy((char *)&buffer[2], robot_host.data(), MAX_HOST_LEN);
        buffer[2 + (MAX_HOST_LEN + 1)]     = robot_port >> 8U;
        buffer[2 + (MAX_HOST_LEN + 1) + 1] = robot_port & 0xFFU;
        return buffer;
    }

    [[nodiscard]] static EpuckHeartbeatPacket unpack(const void *const buffer)
    {
        EpuckHeartbeatPacket packet;
        packet.id       = ((uint8_t *)buffer)[0];
        packet.robot_id = ((uint8_t *)buffer)[1];
        strncpy(packet.robot_host.data(), (char *)&((uint8_t *)buffer)[2], MAX_HOST_LEN);
        packet.robot_port = ((uint8_t *)buffer)[2 + MAX_HOST_LEN] << 8U | ((uint8_t *)buffer)[2 + MAX_HOST_LEN + 1];
        return packet;
    }
};

class PACKED EpuckNeighbourPacket
{
public:
    PACKED uint8_t robot_id                        = 0;
    PACKED std::array<char, MAX_HOST_LEN + 1> host = {0};
    PACKED uint16_t port                           = 0;
    PACKED float dist                              = 0;

    EpuckNeighbourPacket() = default;
    [[nodiscard]] explicit EpuckNeighbourPacket(const void *const buffer) { *this = std::move(unpack(buffer)); }

    bool operator==(const EpuckNeighbourPacket &other) const
    {
        return robot_id == other.robot_id && host == other.host && port == other.port && dist == other.dist;
    }

    bool operator<(const EpuckNeighbourPacket &other) const { return robot_id < other.robot_id; }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckNeighbourPacket)> buffer = {0};
        buffer[0]                                                = robot_id;
        strncpy((char *)&buffer[1], host.data(), MAX_HOST_LEN);
        buffer[1 + (MAX_HOST_LEN + 1)]     = port >> 8U;
        buffer[1 + (MAX_HOST_LEN + 1) + 1] = port & 0xFFU;
        memcpy(&buffer[1 + (MAX_HOST_LEN + 1) + 2], &dist, 4);
        return buffer;
    }

    [[nodiscard]] static EpuckNeighbourPacket unpack(const void *const buffer)
    {
        EpuckNeighbourPacket packet;
        packet.robot_id = ((uint8_t *)buffer)[0];
        strncpy(packet.host.data(), (char *)&((uint8_t *)buffer)[1], MAX_HOST_LEN);
        packet.port =
            ((uint8_t *)buffer)[1 + (MAX_HOST_LEN + 1)] << 8U | ((uint8_t *)buffer)[1 + (MAX_HOST_LEN + 1) + 1];
        memcpy(&packet.dist, &((uint8_t *)buffer)[1 + (MAX_HOST_LEN + 1) + 2], 4);
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
    [[nodiscard]] explicit EpuckHeartbeatResponsePacket(const void *const buffer) { *this = std::move(unpack(buffer)); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckHeartbeatResponsePacket)> buffer = {0};
        buffer[0]                                                        = id;
        buffer[1]                                                        = num_neighbours;
        int i                                                            = 0;
        for (const auto &neighbour : neighbours)
        {
            memcpy(&buffer[2 + i * sizeof(EpuckNeighbourPacket)], neighbour.pack().data(),
                   sizeof(EpuckNeighbourPacket));
            i++;
        }
        return buffer;
    }

    [[nodiscard]] static EpuckHeartbeatResponsePacket unpack(const void *const buffer)
    {
        EpuckHeartbeatResponsePacket packet;
        packet.id             = ((uint8_t *)buffer)[0];
        packet.num_neighbours = ((uint8_t *)buffer)[1];
        for (int i = 0; i < packet.num_neighbours; i++)
        {
            packet.neighbours.at(i) =
                EpuckNeighbourPacket::unpack(&((uint8_t *)buffer)[2 + i * sizeof(EpuckNeighbourPacket)]);
        }
        return packet;
    }
};

class PACKED EpuckKnowledgePacket
{
public:
    PACKED uint8_t id                                = 0x22;
    PACKED uint8_t robot_id                          = 0;
    PACKED uint8_t N                                 = 0;
    PACKED std::array<uint8_t, MAX_ROBOTS> known_ids = {0};

    EpuckKnowledgePacket() = default;
    [[nodiscard]] explicit EpuckKnowledgePacket(const void *const buffer) { *this = std::move(unpack(buffer)); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckKnowledgePacket)> buffer = {0};
        buffer[0]                                                = id;
        buffer[1]                                                = robot_id;
        buffer[2]                                                = N;
        int i                                                    = 0;
        for (const auto &known_id : known_ids)
        {
            buffer[3 + i++] = known_id;
        }
        return buffer;
    }

    [[nodiscard]] static EpuckKnowledgePacket unpack(const void *const buffer)
    {
        EpuckKnowledgePacket packet;
        packet.id       = ((uint8_t *)buffer)[0];
        packet.robot_id = ((uint8_t *)buffer)[1];
        packet.N        = ((uint8_t *)buffer)[2];
        for (int i = 0; i < packet.N; i++)
        {
            packet.known_ids.at(i) = ((uint8_t *)buffer)[3 + i];
        }
        return packet;
    }
};

class PACKED EpuckAddressKnowledgePacket
{
public:
    PACKED uint8_t id                                 = 0x23;
    PACKED uint8_t robot_id                           = 0;
    PACKED uint8_t N                                  = 0;
    PACKED std::array<char, MAX_HOST_LEN + 1> address = {0};
    PACKED std::array<uint8_t, MAX_ROBOTS> known_ids  = {0};

    EpuckAddressKnowledgePacket() = default;
    [[nodiscard]] explicit EpuckAddressKnowledgePacket(const void *const buffer) { *this = std::move(unpack(buffer)); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, sizeof(EpuckAddressKnowledgePacket)> buffer = {0};
        buffer[0]                                                       = id;
        buffer[1]                                                       = robot_id;
        buffer[2]                                                       = N;
        strncpy((char *)&buffer[3], address.data(), MAX_HOST_LEN);
        int i = 0;
        for (const auto &known_id : known_ids)
        {
            buffer.at(3 + (MAX_HOST_LEN + 1) + i++) = known_id;
        }
        return buffer;
    }

    [[nodiscard]] static EpuckAddressKnowledgePacket unpack(const void *const buffer)
    {
        EpuckAddressKnowledgePacket packet;
        packet.id       = ((uint8_t *)buffer)[0];
        packet.robot_id = ((uint8_t *)buffer)[1];
        packet.N        = ((uint8_t *)buffer)[2];
        strncpy(packet.address.data(), &((char *)buffer)[3], MAX_HOST_LEN);
        for (int i = 0; i < packet.N; i++)
        {
            packet.known_ids.at(i) = ((uint8_t *)buffer)[3 + (MAX_HOST_LEN + 1) + i];
        }
        return packet;
    }
};
