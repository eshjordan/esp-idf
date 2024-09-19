
#pragma once

#ifdef __cplusplus
extern "C++" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "types.hpp"
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

class EpuckHeartbeatPacket
{
public:
    uint8_t id                           = 0x20;
    uint8_t robot_id                     = 0;
    HostString robot_host                = {};
    uint16_t robot_port                  = 0;
    static constexpr uint8_t PACKET_SIZE = 1 + 1 + MAX_HOST_LEN + 2;

    void *pack()
    {
        uint8_t buffer[PACKET_SIZE] = {};
        buffer[0]                   = id;
        buffer[1]                   = robot_id;
        strcpy((char *)&buffer[2], robot_host.c_str());
        buffer[2 + MAX_HOST_LEN]     = robot_port >> 8U;
        buffer[2 + MAX_HOST_LEN + 1] = robot_port & 0xFFU;
        return buffer;
    }

    static EpuckHeartbeatPacket unpack(void *buffer)
    {
        EpuckHeartbeatPacket packet = {};
        packet.id                   = ((uint8_t *)buffer)[0];
        packet.robot_id             = ((uint8_t *)buffer)[1];
        packet.robot_host           = &((char *)buffer)[2];
        packet.robot_port = ((uint8_t *)buffer)[2 + MAX_HOST_LEN] << 8U | ((uint8_t *)buffer)[2 + MAX_HOST_LEN + 1];
        return packet;
    }

    static constexpr uint8_t calcsize() { return PACKET_SIZE; }
};

class EpuckNeighbourPacket
{
public:
    uint8_t robot_id                     = 0;
    HostString host                      = {};
    uint16_t port                        = 0;
    float dist                           = 0;
    static constexpr uint8_t PACKET_SIZE = 1 + MAX_HOST_LEN + 2 + 4;

    void *pack()
    {
        uint8_t buffer[PACKET_SIZE] = {};
        buffer[0]                   = robot_id;
        strcpy((char *)&buffer[1], host.c_str());
        buffer[1 + MAX_HOST_LEN]     = port >> 8U;
        buffer[1 + MAX_HOST_LEN + 1] = port & 0xFFU;
        memcpy(&buffer[1 + MAX_HOST_LEN + 2], &dist, 4);
        return buffer;
    }

    static EpuckNeighbourPacket unpack(void *buffer)
    {
        EpuckNeighbourPacket packet = {};
        packet.robot_id             = ((uint8_t *)buffer)[0];
        packet.host                 = &((char *)buffer)[1];
        packet.port = ((uint8_t *)buffer)[1 + MAX_HOST_LEN] << 8U | ((uint8_t *)buffer)[1 + MAX_HOST_LEN + 1];
        memcpy(&packet.dist, &((uint8_t *)buffer)[1 + MAX_HOST_LEN + 2], 4);
        return packet;
    }

    static constexpr uint8_t calcsize() { return EpuckNeighbourPacket::PACKET_SIZE; }
};

class EpuckHeartbeatResponsePacket
{
public:
    uint8_t id             = 0x21;
    uint8_t num_neighbours = 0;
    EpuckNeighbourPacket neighbours[MAX_ROBOTS];
    static constexpr uint8_t PACKET_SIZE = 1 + 1 + MAX_ROBOTS * EpuckNeighbourPacket::PACKET_SIZE;

    void *pack()
    {
        uint8_t buffer[PACKET_SIZE] = {};
        buffer[0]                   = id;
        buffer[1]                   = num_neighbours;
        for (int i = 0; i < num_neighbours; i++)
        {
            memcpy(&buffer[2 + i * EpuckNeighbourPacket::PACKET_SIZE], neighbours[i].pack(),
                   EpuckNeighbourPacket::PACKET_SIZE);
        }
        return buffer;
    }

    static EpuckHeartbeatResponsePacket unpack(void *buffer)
    {
        EpuckHeartbeatResponsePacket packet = {};
        packet.id                           = ((uint8_t *)buffer)[0];
        packet.num_neighbours               = ((uint8_t *)buffer)[1];
        for (int i = 0; i < packet.num_neighbours; i++)
        {
            packet.neighbours[i] =
                EpuckNeighbourPacket::unpack(&((uint8_t *)buffer)[2 + i * EpuckNeighbourPacket::PACKET_SIZE]);
        }
        return packet;
    }

    static constexpr uint8_t calcsize() { return PACKET_SIZE; }
};

class EpuckKnowledgePacket
{
public:
    uint8_t id                           = 0x22;
    uint8_t robot_id                     = 0;
    uint8_t N                            = 0;
    uint8_t known_ids[MAX_ROBOTS]        = {};
    static constexpr uint8_t PACKET_SIZE = 1 + 1 + 1 + MAX_ROBOTS;

    void *pack()
    {
        uint8_t buffer[PACKET_SIZE] = {};
        buffer[0]                   = id;
        buffer[1]                   = robot_id;
        buffer[2]                   = N;
        for (int i = 0; i < N; i++)
        {
            buffer[3 + i] = known_ids[i];
        }
        return buffer;
    }

    static EpuckKnowledgePacket unpack(void *buffer)
    {
        EpuckKnowledgePacket packet = {};
        packet.id                   = ((uint8_t *)buffer)[0];
        packet.robot_id             = ((uint8_t *)buffer)[1];
        packet.N                    = ((uint8_t *)buffer)[2];
        for (int i = 0; i < packet.N; i++)
        {
            packet.known_ids[i] = ((uint8_t *)buffer)[3 + i];
        }
        return packet;
    }

    static constexpr uint8_t calcsize() { return PACKET_SIZE; }
};

class EpuckAddressKnowledgePacket
{
public:
    uint8_t id                           = 0x23;
    uint8_t robot_id                     = 0;
    uint8_t N                            = 0;
    HostString address                   = {};
    uint8_t known_ids[MAX_ROBOTS]        = {};
    static constexpr uint8_t PACKET_SIZE = 1 + 1 + 1 + MAX_HOST_LEN + MAX_ROBOTS;

    void *pack()
    {
        uint8_t buffer[PACKET_SIZE] = {};
        buffer[0]                   = id;
        buffer[1]                   = robot_id;
        buffer[2]                   = N;
        strcpy((char *)&buffer[3], address.c_str());
        for (int i = 0; i < N; i++)
        {
            buffer[3 + MAX_HOST_LEN + i] = known_ids[i];
        }
        return buffer;
    }

    static EpuckAddressKnowledgePacket unpack(void *buffer)
    {
        EpuckAddressKnowledgePacket packet = {};
        packet.id                          = ((uint8_t *)buffer)[0];
        packet.robot_id                    = ((uint8_t *)buffer)[1];
        packet.N                           = ((uint8_t *)buffer)[2];
        packet.address                     = &((char *)buffer)[3];
        for (int i = 0; i < packet.N; i++)
        {
            packet.known_ids[i] = ((uint8_t *)buffer)[3 + MAX_HOST_LEN + i];
        }
        return packet;
    }

    static constexpr uint8_t calcsize() { return PACKET_SIZE; }
};

#ifdef __cplusplus
}
#endif
