
#pragma once

#include "types.hpp"
#ifdef __cplusplus
extern "C++" {
#endif

#include "EpuckPackets.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <set>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <vector>

template <typename T, typename U> class RobotCommsModel;
class BaseKnowledgeClient;

class BaseKnowledgeServer
{
private:
    RobotCommsModel<BaseKnowledgeServer, BaseKnowledgeClient> *robot_model;

public:
    explicit BaseKnowledgeServer(RobotCommsModel<BaseKnowledgeServer, BaseKnowledgeClient> *robot_model)
        : robot_model(robot_model){};
    virtual void start() = 0;
    virtual void stop()  = 0;
};

class BaseKnowledgeClient
{
private:
    EpuckNeighbourPacket neighbour;
    bool (*running)();
    RobotCommsModel<BaseKnowledgeServer, BaseKnowledgeClient> *robot_model;

public:
    BaseKnowledgeClient(EpuckNeighbourPacket neighbour, bool (*running)(),
                        RobotCommsModel<BaseKnowledgeServer, BaseKnowledgeClient> *robot_model)
        : neighbour(neighbour), running(running), robot_model(robot_model){};
    virtual void start() = 0;
    virtual void stop()  = 0;
};

template <typename T, typename U> class RobotCommsModel
{
    static_assert(std::is_base_of<BaseKnowledgeServer, T>::value, "T must inherit from BaseKnowledgeServer");
    static_assert(std::is_base_of<BaseKnowledgeClient, U>::value, "U must inherit from BaseKnowledgeClient");

private:
    uint8_t robot_id        = 0;
    HostString manager_host = {0};
    uint16_t manager_port   = 0;
    HostString robot_host   = {0};
    uint16_t robot_port     = 0;

    alignas(T) uint8_t knowledge_server_buffer[sizeof(T)];
    T *knowledge_server;
    std::vector<U, RobotSizeAllocator<U>> knowledge_clients;

    std::set<uint8_t, std::less<uint8_t>, RobotSizeAllocator<uint8_t>> known_ids;

public:
    RobotCommsModel(const uint8_t &robot_id, const HostString &manager_host, const uint16_t &manager_port,
                    const HostString &robot_host, const uint16_t &robot_port)
    {
        this->robot_id     = robot_id;
        this->manager_port = manager_port;
        this->robot_port   = robot_port;
        this->known_ids    = std::set<uint8_t, std::less<uint8_t>, RobotSizeAllocator<uint8_t>>();
    }

    ~RobotCommsModel() { this->stop(); }

    void start();
    void stop();
    void exchange_heartbeats()
    {
        this->knowledge_server = new (this->knowledge_server_buffer) T(this);
        while (true)
        {
            if (false) { this->knowledge_clients.push_back(T(this)); }
        }
    }
};

#ifdef __cplusplus
}
#endif
