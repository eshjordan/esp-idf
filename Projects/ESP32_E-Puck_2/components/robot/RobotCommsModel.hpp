
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
#include <set>
#include <stdint.h>
#include <sys/time.h>
#include <vector>

template <typename T, typename U> class RobotCommsModel;
class BaseKnowledgeClient;

class BaseKnowledgeServer
{
private:
    std::shared_ptr<RobotCommsModel<BaseKnowledgeServer, BaseKnowledgeClient>> robot_model;

public:
    explicit BaseKnowledgeServer(std::shared_ptr<RobotCommsModel<BaseKnowledgeServer, BaseKnowledgeClient>> robot_model)
        : robot_model(robot_model){};
    virtual void start() = 0;
    virtual void stop()  = 0;
};

class BaseKnowledgeClient
{
private:
    EpuckNeighbourPacket neighbour;
    bool (*running)();
    std::shared_ptr<RobotCommsModel<BaseKnowledgeServer, BaseKnowledgeClient>> robot_model;

public:
    BaseKnowledgeClient(EpuckNeighbourPacket neighbour, bool (*running)(),
                        std::shared_ptr<RobotCommsModel<BaseKnowledgeServer, BaseKnowledgeClient>> robot_model)
        : neighbour(neighbour), running(running), robot_model(robot_model){};
    virtual void start() = 0;
    virtual void stop()  = 0;
};

template <typename T, typename U> class RobotCommsModel : public std::enable_shared_from_this<RobotCommsModel<T, U>>
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
    std::unique_ptr<T, void (*)(T *)> knowledge_server;
    std::map<uint8_t, U, std::less<uint8_t>, RobotSizeAllocator<std::pair<uint8_t, U>>> knowledge_clients;

    bool run_heartbeats = false;
#ifdef ESP32
    TaskHandle_t heartbeat_thread = NULL;
#else
    std::thread heartbeat_thread = std::thread();
#endif

public:
    RobotCommsModel(const uint8_t &robot_id, const HostString &manager_host, const uint16_t &manager_port,
                    const HostString &robot_host, const uint16_t &robot_port)
    {
        this->robot_id     = robot_id;
        this->manager_port = manager_port;
        this->robot_port   = robot_port;
    }

    ~RobotCommsModel() { this->stop(); }

    void start()
    {
        this->knowledge_server = std::unique_ptr<T, void (*)(T *)>(new (this->knowledge_server_buffer) T(this->shared_from_this()), [](T *p){ delete static_cast<T *>(p); });
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
        auto heartbeat_client = asio::ip::udp::socket(io_context);

        auto manager_endpoint =
            asio::ip::udp::endpoint(asio::ip::address::from_string(this->manager_host), this->manager_port);

        while (this->run_heartbeats)
        {
            auto packet       = EpuckHeartbeatPacket();
            packet.robot_id   = this->robot_id;
            packet.robot_host = this->robot_host;
            packet.robot_port = this->robot_port;

            heartbeat_client.send_to(asio::buffer(packet.pack(), packet.calcsize()), manager_endpoint);

            struct pollfd pfd = {.fd = heartbeat_client.native_handle(), .events = POLLIN, .revents = 0};
            int retval        = poll(&pfd, 1, 10);
            if (retval == 0)
            { // timeout
                continue;
            }
            if (retval < 0)
            {
                perror("poll");
                continue;
            }

            auto response_buffer = reinterpret_cast<uint8_t *>(EpuckHeartbeatResponsePacket().pack());

            heartbeat_client.receive_from(asio::buffer(response_buffer, 2), manager_endpoint);
            auto num_neighbours = response_buffer[1];
            heartbeat_client.receive_from(
                asio::buffer(response_buffer + 2, num_neighbours * EpuckNeighbourPacket::calcsize()), manager_endpoint);
            auto response = EpuckHeartbeatResponsePacket::unpack(response_buffer);

            // Connect to new robots that are listed in the response if they have a lower ID
            for (const auto &neighbour : response.neighbours)
            {
                // Only connect to robots with lower IDs that are not already connected
                if (this->knowledge_clients.find(neighbour.robot_id) != this->knowledge_clients.end()
                    || neighbour.robot_id >= this->robot_id)
                {
                    continue;
                }

                this->knowledge_clients.emplace(
                    neighbour.robot_id, neighbour,
                    [&]() { return this->knowledge_clients.find(neighbour.robot_id) != this->knowledge_clients.end(); },
                    this->shared_from_this());
                this->knowledge_clients[neighbour.robot_id].start();
            }

            // Disconnect from connected robots that are not listed in the response
            for (const auto &it : this->knowledge_clients)
            {
                auto &neighbour_id = it.first;
                if (std::find(response.neighbours.begin(), response.neighbours.end(), neighbour_id)
                    != response.neighbours.end())
                {
                    continue;
                }
                this->knowledge_clients[neighbour_id].stop();
                this->knowledge_clients.erase(neighbour_id);
            }

            // Sleep for 1 second
#ifdef ESP32
            vTaskDelay(1000 / portTICK_PERIOD_MS);
#else
            std::this_thread::sleep_for(std::chrono::seconds(1));
#endif
        }
    }
};

#ifdef __cplusplus
}
#endif
