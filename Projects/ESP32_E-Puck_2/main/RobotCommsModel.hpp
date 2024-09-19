
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "EpuckPackets.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include <cstddef>
#include <functional>
#include <memory>
#include <set>
#include <sys/ioctl.h>
#include <sys/socket.h>

template <typename BaseKnowledgeServer, typename BaseKnowledgeClient> class RobotCommsModel;
class BaseKnowledgeClient;

class BaseKnowledgeServer
{
private:
    RobotCommsModel<BaseKnowledgeServer, BaseKnowledgeClient> *robot_model;

public:
    BaseKnowledgeServer(RobotCommsModel<BaseKnowledgeServer, BaseKnowledgeClient> *robot_model)
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

template <typename BaseKnowledgeServer, typename BaseKnowledgeClient> class RobotCommsModel
{
private:
    uint8_t robot_id                = 0;
    char manager_host[MAX_HOST_LEN] = {0};
    uint16_t manager_port           = 0;
    char robot_host[MAX_HOST_LEN]   = {0};
    uint16_t robot_port             = 0;
    BaseKnowledgeServer *knowledge_server;
    BaseKnowledgeClient *knowledge_client;

    template <std::size_t Size> class StaticMemoryAllocator
    {
        static char buffer_[Size];
        static std::size_t offset_;

    public:
        template <typename T> static auto allocate()
        {
            const auto new_offset = offset_ + sizeof(T);
            const auto place      = buffer_ + new_offset - sizeof(T);
            offset_               = new_offset;
            return new (place) T{};
        }
    };

    std::set<uint8_t, std::less<uint8_t>, StaticMemoryAllocator<MAX_ROBOTS>> known_ids;

public:
    RobotCommsModel(uint8_t robot_id, char *manager_host, uint16_t manager_port, char *robot_host, uint16_t robot_port,
                    BaseKnowledgeServer *knowledge_server, BaseKnowledgeClient *knowledge_client)
        : robot_id(robot_id), manager_port(manager_port), robot_port(robot_port), knowledge_server(knowledge_server),
          knowledge_client(knowledge_client)
    {
        this->known_ids = std::set<uint8_t, std::less<uint8_t>, StaticMemoryAllocator<MAX_ROBOTS>>();
    }
    ~RobotCommsModel();

    void start();
    void stop();
    void exchange_heartbeats();
};

#ifdef __cplusplus
}
#endif
