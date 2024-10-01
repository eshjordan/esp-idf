#pragma once

#include "types.hpp"
#include <array>
#include <cstdint>
#include <string.h>

class EpuckHeartbeatPacket
{
public:
    uint8_t id                           = 0x20;
    uint8_t robot_id                     = 0;
    HostSizeString robot_host            = {};
    uint16_t robot_port                  = 0;
    static constexpr size_t PACKET_SIZE = 1 + 1 + (MAX_HOST_LEN+1) + 2;

    EpuckHeartbeatPacket() = default;
    [[nodiscard]] explicit EpuckHeartbeatPacket(const void *const buffer) { *this = std::move(unpack(buffer)); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, PACKET_SIZE> buffer = {0};
        buffer[0]                               = id;
        buffer[1]                               = robot_id;
        strcpy((char *)&buffer[2], robot_host.c_str());
        buffer[2 + (MAX_HOST_LEN+1)]     = robot_port >> 8U;
        buffer[2 + (MAX_HOST_LEN+1) + 1] = robot_port & 0xFFU;
        return buffer;
    }

    [[nodiscard]] static EpuckHeartbeatPacket unpack(const void *const buffer)
    {
        EpuckHeartbeatPacket packet;
        packet.id         = ((uint8_t *)buffer)[0];
        packet.robot_id   = ((uint8_t *)buffer)[1];
        packet.robot_host = &((char *)buffer)[2];
        packet.robot_port = ((uint8_t *)buffer)[2 + MAX_HOST_LEN] << 8U | ((uint8_t *)buffer)[2 + MAX_HOST_LEN + 1];
        return packet;
    }

    static constexpr size_t calcsize() { return PACKET_SIZE; }
};

class EpuckNeighbourPacket
{
public:
    uint8_t robot_id                     = 0;
    HostSizeString host                  = {};
    uint16_t port                        = 0;
    float dist                           = 0;
    static constexpr size_t PACKET_SIZE = 1 + (MAX_HOST_LEN+1) + 2 + 4;

    EpuckNeighbourPacket() = default;
    [[nodiscard]] explicit EpuckNeighbourPacket(const void *const buffer) { *this = std::move(unpack(buffer)); }

    bool operator==(const EpuckNeighbourPacket &other) const
    {
        return robot_id == other.robot_id && host == other.host && port == other.port && dist == other.dist;
    }

    bool operator<(const EpuckNeighbourPacket &other) const { return robot_id < other.robot_id; }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, PACKET_SIZE> buffer = {0};
        buffer[0]                               = robot_id;
        strcpy((char *)&buffer[1], host.c_str());
        buffer[1 + (MAX_HOST_LEN+1)]     = port >> 8U;
        buffer[1 + (MAX_HOST_LEN+1) + 1] = port & 0xFFU;
        memcpy(&buffer[1 + (MAX_HOST_LEN+1) + 2], &dist, 4);
        return buffer;
    }

    [[nodiscard]] static EpuckNeighbourPacket unpack(const void *const buffer)
    {
        EpuckNeighbourPacket packet;
        packet.robot_id = ((uint8_t *)buffer)[0];
        packet.host     = &((char *)buffer)[1];
        packet.port     = ((uint8_t *)buffer)[1 + (MAX_HOST_LEN+1)] << 8U | ((uint8_t *)buffer)[1 + (MAX_HOST_LEN+1) + 1];
        memcpy(&packet.dist, &((uint8_t *)buffer)[1 + (MAX_HOST_LEN+1) + 2], 4);
        return packet;
    }

    static constexpr size_t calcsize() { return PACKET_SIZE; }
};

class EpuckHeartbeatResponsePacket
{
public:
    uint8_t id             = 0x21;
    uint8_t num_neighbours = 0;
    RobotSizeSet<EpuckNeighbourPacket> neighbours;
    static constexpr size_t PACKET_SIZE = 1 + 1 + MAX_ROBOTS * EpuckNeighbourPacket::calcsize();

    EpuckHeartbeatResponsePacket() = default;
    [[nodiscard]] explicit EpuckHeartbeatResponsePacket(const void *const buffer) { *this = std::move(unpack(buffer)); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, PACKET_SIZE> buffer = {0};
        buffer[0]                               = id;
        buffer[1]                               = num_neighbours;
        int i                                   = 0;
        for (const auto &neighbour : neighbours)
        {
            memcpy(&buffer[2 + i * EpuckNeighbourPacket::calcsize()], neighbour.pack().data(),
                   EpuckNeighbourPacket::calcsize());
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
            packet.neighbours.emplace(&((uint8_t *)buffer)[2 + i * EpuckNeighbourPacket::calcsize()]);
        }
        return packet;
    }

    static constexpr size_t calcsize() { return PACKET_SIZE; }
};

class EpuckKnowledgePacket
{
public:
    uint8_t id       = 0x22;
    uint8_t robot_id = 0;
    uint8_t N        = 0;
    RobotSizeSet<uint8_t> known_ids;
    static constexpr size_t PACKET_SIZE = 1 + 1 + 1 + MAX_ROBOTS;

    EpuckKnowledgePacket() = default;
    [[nodiscard]] explicit EpuckKnowledgePacket(const void *const buffer) { *this = std::move(unpack(buffer)); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, PACKET_SIZE> buffer = {0};
        buffer[0]                               = id;
        buffer[1]                               = robot_id;
        buffer[2]                               = N;
        int i                                   = 0;
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
            packet.known_ids.emplace(((uint8_t *)buffer)[3 + i]);
        }
        return packet;
    }

    static constexpr size_t calcsize() { return PACKET_SIZE; }
};

class EpuckAddressKnowledgePacket
{
public:
    uint8_t id             = 0x23;
    uint8_t robot_id       = 0;
    uint8_t N              = 0;
    HostSizeString address = {};
    RobotSizeSet<uint8_t> known_ids;
    static constexpr size_t PACKET_SIZE = 1 + 1 + 1 + (MAX_HOST_LEN+1) + MAX_ROBOTS;

    EpuckAddressKnowledgePacket() = default;
    [[nodiscard]] explicit EpuckAddressKnowledgePacket(const void *const buffer) { *this = std::move(unpack(buffer)); }

    [[nodiscard]] auto pack() const
    {
        std::array<uint8_t, PACKET_SIZE> buffer = {0};
        buffer[0]                               = id;
        buffer[1]                               = robot_id;
        buffer[2]                               = N;
        strcpy((char *)&buffer[3], address.c_str());
        int i = 0;
        for (const auto &known_id : known_ids)
        {
            buffer.at(3 + (MAX_HOST_LEN+1) + i++) = known_id;
        }
        return buffer;
    }

    [[nodiscard]] static EpuckAddressKnowledgePacket unpack(const void *const buffer)
    {
        EpuckAddressKnowledgePacket packet;
        packet.id       = ((uint8_t *)buffer)[0];
        packet.robot_id = ((uint8_t *)buffer)[1];
        packet.N        = ((uint8_t *)buffer)[2];
        packet.address  = &((char *)buffer)[3];
        for (int i = 0; i < packet.N; i++)
        {
            packet.known_ids.emplace(((uint8_t *)buffer)[3 + (MAX_HOST_LEN+1) + i]);
        }
        return packet;
    }

    static constexpr size_t calcsize() { return PACKET_SIZE; }
};
