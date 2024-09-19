#pragma once

#ifdef __cplusplus
extern "C++" {
#endif

#include "RobotCommsModel.hpp"

class UDPKnowledgeServer : public BaseKnowledgeServer
{
public:
    template <typename T, typename U> explicit UDPKnowledgeServer(RobotCommsModel<T, U> *robot_model)
        : BaseKnowledgeServer(robot_model){};
    void start() {}
    void stop() {}
};

class UDPKnowledgeClient : public BaseKnowledgeClient
{
public:
    template <typename T, typename U>
    UDPKnowledgeClient(EpuckNeighbourPacket neighbour, bool (*running)(), RobotCommsModel<T, U> *robot_model)
        : BaseKnowledgeClient(neighbour, running, robot_model){};
    void start() {}
    void stop() {}
};

#ifdef __cplusplus
}
#endif
