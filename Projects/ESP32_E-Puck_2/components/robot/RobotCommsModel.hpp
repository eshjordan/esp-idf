
#pragma once

#include "types.hpp"
#ifdef __cplusplus
extern "C++" {
#endif

#define ESP32

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#else
#include <thread>
#endif

#include "EpuckPackets.hpp"
#include <asio.hpp>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <poll.h>
#include <set>
#include <stdint.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
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
    std::map<uint8_t, U, std::less<uint8_t>, RobotSizeAllocator<std::pair<uint8_t, U>>> knowledge_clients;

    std::set<uint8_t, std::less<uint8_t>, RobotSizeAllocator<uint8_t>> known_ids;

    bool run_heartbeats = false;
#ifdef ESP32
    TaskHandle_t heartbeat_thread = NULL;
#else
    std::thread heartbeat_thread = std::thread();
#endif

    asio::ip::udp::socket heartbeat_client = 0;

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

    void start()
    {
        this->knowledge_server = new (this->knowledge_server_buffer) T(this);
        this->knowledge_server->start();

        this->knowledge_clients.clear();

        this->run_heartbeats = true;
#ifdef ESP32
        xTaskCreatePinnedToCore(&RobotCommsModel::exchange_heartbeats, "exchange_heartbeats", 4096, this, 5,
                                &this->heartbeat_thread, 1);
#else
        this->heartbeat_thread = std::thread(&RobotCommsModel::exchange_heartbeats, this);
#endif
    }

    void stop()
    {
        this->run_heartbeats = false;
#ifdef ESP32
        vTaskDelete(this->heartbeat_thread);
#else
        this->heartbeat_thread.join();
#endif
    }

    void exchange_heartbeats()
    {
        asio::io_context io_context;
        this->heartbeat_client = asio::ip::udp::socket socket(io_context);
        this->heartbeat_client.open(asio::ip::udp::v4());

        auto manager_endpoint =
            asio::ip::udp::endpoint(asio::ip::address::from_string(this->manager_host), this->manager_port);

        while (this->run_heartbeats)
        {
            auto packet       = EpuckHeartbeatPacket();
            packet.robot_id   = this->robot_id;
            packet.robot_host = this->robot_host;
            packet.robot_port = this->robot_port;

            this->heartbeat_client.sendto(asio::buffer(packet.pack(), packet.calcsize()), manager_endpoint);

            struct pollfd pfd = {.fd = this->heartbeat_client.native_handle(), .events = POLLIN, .revents = 0};
            int retval = poll(&pfd, 1, 10);
            if (retval == 0) { // timeout
                continue;
            }
            if (retval < 0) {
                perror("poll");
                continue;
            }

            auto response = EpuckHeartbeatResponsePacket().pack();

            this->heartbeat_client.receive_from(asio::buffer(response.pack(), packet.calcsize()), manager_endpoint);
        }
    }
};

#ifdef __cplusplus
}
#endif
