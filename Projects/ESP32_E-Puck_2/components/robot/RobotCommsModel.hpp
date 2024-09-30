
#pragma once

#include <asio.hpp>

#include "EpuckPackets.hpp"
#include "types.hpp"
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <stdint.h>
#include <sys/time.h>
#include <thread>
#include <utility>
#include <vector>

class BaseRobotCommsModel
{
public:
    using robot_id_type      = uint8_t;
    using known_ids_type     = RobotSizeSet<robot_id_type>;
    using known_ids_iterator = known_ids_type::iterator;

    const robot_id_type robot_id;

    explicit BaseRobotCommsModel(const robot_id_type &robot_id) : robot_id(robot_id)
    {
        this->known_ids.insert(robot_id);
    }

    virtual void Start() = 0;
    virtual void Stop()  = 0;

    [[nodiscard]] std::pair<const known_ids_iterator&, const known_ids_iterator&> GetKnownIds() const
    {
        return {this->known_ids.begin(), this->known_ids.end()};
    }

    size_t InsertKnownIds(known_ids_iterator first, known_ids_iterator last) {
        auto size_before = this->known_ids.size();
        this->known_ids.insert(first, last);
        return this->known_ids.size() - size_before;
    }

protected:
    known_ids_type known_ids = {};
};

class BaseKnowledgeServer
{
protected:
    std::shared_ptr<BaseRobotCommsModel> robot_model;

public:
    BaseKnowledgeServer() = default;
    explicit BaseKnowledgeServer(std::shared_ptr<BaseRobotCommsModel> robot_model)
        : robot_model(std::move(robot_model)){};
    virtual void Start() = 0;
    virtual void Stop()  = 0;
};

class BaseKnowledgeClient
{
protected:
    EpuckNeighbourPacket neighbour;
    std::function<bool()> running;
    std::shared_ptr<BaseRobotCommsModel> robot_model;

public:
    BaseKnowledgeClient() = default;
    BaseKnowledgeClient(EpuckNeighbourPacket neighbour, std::function<bool()> running,
                        std::shared_ptr<BaseRobotCommsModel> robot_model)
        : neighbour(std::move(neighbour)), running(running), robot_model(std::move(robot_model)){};
    virtual void Start() = 0;
    virtual void Stop()  = 0;
};

template <typename T, typename U> class RobotCommsModel : public std::enable_shared_from_this<RobotCommsModel<T, U>>,
                                                          BaseRobotCommsModel
{
    static_assert(std::is_base_of_v<BaseKnowledgeServer, T>, "T must inherit from BaseKnowledgeServer");
    static_assert(std::is_base_of_v<BaseKnowledgeClient, U>, "U must inherit from BaseKnowledgeClient");

public:
    RobotCommsModel(const robot_id_type &robot_id, HostSizeString manager_host, const uint16_t &manager_port,
                    HostSizeString robot_host, const uint16_t &robot_port)
        : BaseRobotCommsModel(robot_id), manager_host_(std::move(manager_host)), manager_port_(manager_port),
          robot_host_(std::move(robot_host)), robot_port_(robot_port)
    {
    }

    ~RobotCommsModel() { this->Stop(); }

    void Start() override
    {
        this->knowledge_server_ = std::shared_ptr<T>(new (this->knowledge_server_buffer_) T(
            std::reinterpret_pointer_cast<BaseRobotCommsModel>(this->shared_from_this())));
        this->knowledge_server_->Start();

        this->knowledge_clients_.clear();

        this->run_heartbeats_ = true;
        std::thread tmp_thread(&RobotCommsModel::ExchangeHeartbeats, this);
        this->heartbeat_thread_.swap(tmp_thread);
    }

    void Stop() override
    {
        this->run_heartbeats_ = false;
        this->heartbeat_thread_.join();
    }

private:
    void ExchangeHeartbeats()
    {
        asio::io_context io_context;
        auto heartbeat_client = asio::ip::udp::socket(io_context);

        auto manager_endpoint =
            asio::ip::udp::endpoint(asio::ip::address::from_string(this->manager_host_.c_str()), this->manager_port_);

        while (this->run_heartbeats_)
        {
            ESP_LOGD(RobotCommsModel::TAG, "Sending heartbeat to %s:%hu", manager_host_.c_str(), this->manager_port_);

            auto packet       = EpuckHeartbeatPacket();
            packet.robot_id   = this->robot_id;
            packet.robot_host = this->robot_host_;
            packet.robot_port = this->robot_port_;

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

            for (const auto &neighbour : response.neighbours)
            {
                ESP_LOGD(TAG, "Received neighbour: %hhu (%s:%hu) at distance %f", neighbour.robot_id,
                         neighbour.host.c_str(), neighbour.port, neighbour.dist);
                this->known_ids.insert(neighbour.robot_id);
            }

            // Connect to new robots that are listed in the response if they have a lower ID
            for (const auto &neighbour : response.neighbours)
            {
                // Only connect to robots with lower IDs that are not already connected
                if (this->knowledge_clients_.find(neighbour.robot_id) != this->knowledge_clients_.end()
                    || neighbour.robot_id >= this->robot_id)
                {
                    continue;
                }

                ESP_LOGI(TAG, "Starting thread for neighbour %hhu (%s:%hu)", neighbour.robot_id, neighbour.host.c_str(),
                         neighbour.port);

                auto client = U(
                    neighbour, [&]() { return this->run_heartbeats_; },
                    std::reinterpret_pointer_cast<BaseRobotCommsModel>(this->shared_from_this()));
                this->knowledge_clients_.insert(std::make_pair(neighbour.robot_id, client));
                this->knowledge_clients_[neighbour.robot_id].Start();
            }

            // Disconnect from connected robots that are not listed in the response
            for (const auto &[neighbour_id, _] : this->knowledge_clients_)
            {
                if (std::find_if(response.neighbours.begin(), response.neighbours.end(),
                                 [neighbour_id](auto &neighbour) { return neighbour.robot_id == neighbour_id; })
                    != response.neighbours.end())
                {
                    continue;
                }

                ESP_LOGI(TAG, "Stopping thread for neighbour %hhu", neighbour_id);

                this->knowledge_clients_[neighbour_id].Stop();
                this->knowledge_clients_.erase(neighbour_id);
            }

            // Sleep for 1 second
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    HostSizeString manager_host_ = "";
    uint16_t manager_port_       = 0;
    HostSizeString robot_host_   = "";
    uint16_t robot_port_         = 0;

    alignas(T) uint8_t knowledge_server_buffer_[sizeof(T)] = {0};
    std::shared_ptr<T> knowledge_server_;
    RobotSizeMap<robot_id_type, U> knowledge_clients_;

    bool run_heartbeats_ = false;
    std::thread heartbeat_thread_;

    static constexpr char TAG[] = "RobotCommsModel";
};
