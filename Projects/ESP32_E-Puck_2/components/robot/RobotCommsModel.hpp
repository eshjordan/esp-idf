
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
    const HostSizeString manager_host;
    const uint16_t manager_port;
    const HostSizeString robot_host;
    const uint16_t robot_port;

    explicit BaseRobotCommsModel(const robot_id_type &robot_id, HostSizeString manager_host,
                                 const uint16_t &manager_port, HostSizeString robot_host, const uint16_t &robot_port)
        : robot_id(robot_id), manager_host(std::move(manager_host)), manager_port(manager_port),
          robot_host(std::move(robot_host)), robot_port(robot_port)
    {
        this->known_ids.insert(robot_id);
    }

    virtual void Start() = 0;
    virtual void Stop()  = 0;

    [[nodiscard]] RobotSizeSet<uint8_t> GetKnownIds() const { return {this->known_ids}; }

    size_t InsertKnownIds(const RobotSizeSet<uint8_t> &ids)
    {
        auto size_before = this->known_ids.size();
        this->known_ids.insert(ids.begin(), ids.end());
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
        : BaseRobotCommsModel(robot_id, manager_host, manager_port, robot_host, robot_port)
    {
    }

    ~RobotCommsModel() { this->Stop(); }

    void Start() override
    {
        this->knowledge_server_ = new (this->knowledge_server_buffer_.data()) // NOLINT(cppcoreguidelines-owning-memory)
            T(std::reinterpret_pointer_cast<BaseRobotCommsModel>(this->shared_from_this()));
        this->knowledge_server_->Start();

        this->knowledge_clients_.clear();

        this->run_heartbeats_ = true;
        std::thread tmp_thread(&RobotCommsModel::ExchangeHeartbeats, this);
        this->heartbeat_thread_.swap(tmp_thread);
    }

    void Stop() override
    {
        this->run_heartbeats_ = false;
        if (this->heartbeat_thread_.joinable()) { this->heartbeat_thread_.join(); }
        this->knowledge_server_->Stop();
        this->knowledge_server_->~T();
        for (auto &[_, client] : this->knowledge_clients_)
        {
            client.Stop();
        }
        this->knowledge_clients_.clear();
    }

private:
    void ExchangeHeartbeats()
    {
        auto heartbeat_client = asio::ip::udp::socket(io_context_);
        heartbeat_client.open(asio::ip::udp::v4());

        auto manager_endpoint =
            asio::ip::udp::endpoint(asio::ip::address::from_string(this->manager_host.c_str()), this->manager_port);

        while (this->run_heartbeats_)
        {
            ESP_LOGD(RobotCommsModel::TAG, "Sending heartbeat to %s:%hu", this->manager_host.c_str(),
                     this->manager_port);

            auto packet       = EpuckHeartbeatPacket();
            packet.robot_id   = this->robot_id;
            packet.robot_host = this->robot_host;
            packet.robot_port = this->robot_port;

            ESP_LOGI(TAG, "(%s:%hu)", manager_endpoint.address().to_string().c_str(), manager_endpoint.port());
            std::array<uint8_t, EpuckHeartbeatPacket::PACKET_SIZE> packed_packet = packet.pack();
            heartbeat_client.send_to(asio::buffer(packed_packet, EpuckHeartbeatPacket::calcsize()), manager_endpoint);

            struct pollfd pfd = {heartbeat_client.native_handle(), POLLIN, 0};
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

            auto response_buffer = EpuckHeartbeatResponsePacket().pack();

            heartbeat_client.receive_from(
                asio::buffer(response_buffer.data(), EpuckHeartbeatResponsePacket::calcsize()), manager_endpoint);

            auto response = EpuckHeartbeatResponsePacket::unpack(response_buffer.data());

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
                    neighbour,
                    [&]() {
                        return this->knowledge_clients_.find(neighbour.robot_id) != this->knowledge_clients_.end();
                    },
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

    asio::io_context io_context_;
    alignas(T) std::array<uint8_t, sizeof(T)> knowledge_server_buffer_ = {0};
    T *knowledge_server_;
    RobotSizeMap<robot_id_type, U> knowledge_clients_;

    bool run_heartbeats_ = false;
    std::thread heartbeat_thread_;

    static constexpr char TAG[] = "RobotCommsModel";
};
