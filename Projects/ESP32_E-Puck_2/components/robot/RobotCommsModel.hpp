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

static inline auto known_ids_set_to_string(const RobotSizeSet<robot_id_type> &known_ids)
{
    std::array<char, ((sizeof("65535") - 1) * MAX_ROBOTS) + ((sizeof(", ") - 1) * (MAX_ROBOTS - 1)) + sizeof("")>
        output = {0};
    if (known_ids.empty())
    {
        snprintf(output.data(), sizeof("{}"), "{}");
        return output;
    }

    snprintf(output.data(), sizeof(output), ROBOT_ID_TYPE_FMT, (*known_ids.begin()));
    for (auto it = known_ids.begin(); ++it != known_ids.end();)
    {
        std::array<char, sizeof(", 65535")> buf = {0};
        snprintf(buf.data(), sizeof(buf), ", " ROBOT_ID_TYPE_FMT, (*it));
        strncat(output.data(), buf.data(), sizeof(buf));
    }
    return output;
}

static inline auto known_ids_list_to_string(const std::array<robot_id_type, MAX_ROBOTS> &known_ids, uint8_t N)
{
    std::array<char, ((sizeof("65535") - 1) * MAX_ROBOTS) + ((sizeof(", ") - 1) * (MAX_ROBOTS - 1)) + sizeof("")>
        output = {0};
    if (known_ids.empty())
    {
        snprintf(output.data(), sizeof("{}"), "{}");
        return output;
    }

    snprintf(output.data(), sizeof(output), ROBOT_ID_TYPE_FMT, known_ids[0]);
    for (size_t i = 1; i < N; i++)
    {
        std::array<char, sizeof(", 65535")> buf = {0};
        snprintf(buf.data(), sizeof(buf), ", " ROBOT_ID_TYPE_FMT, known_ids[i]);
        strncat(output.data(), buf.data(), sizeof(buf));
    }
    return output;
}

class BaseRobotCommsModel
{
public:
    using known_ids_type     = RobotSizeSet<robot_id_type>;
    using known_ids_iterator = known_ids_type::iterator;

    const robot_id_type robot_id;
    const HostSizeString manager_host;
    const uint16_t manager_port;
    const HostSizeString robot_host;
    const uint16_t robot_knowledge_exchange_port;
    const uint16_t robot_knowledge_request_port;

    explicit BaseRobotCommsModel(const robot_id_type &robot_id, HostSizeString manager_host,
                                 const uint16_t &manager_port, HostSizeString robot_host,
                                 const uint16_t &robot_knowledge_exchange_port,
                                 const uint16_t &robot_knowledge_request_port)
        : robot_id(robot_id), manager_host(std::move(manager_host)), manager_port(manager_port),
          robot_host(std::move(robot_host)), robot_knowledge_exchange_port(robot_knowledge_exchange_port),
          robot_knowledge_request_port(robot_knowledge_request_port)
    {
        this->known_ids_.insert(robot_id);
    }

    virtual void Start() = 0;
    virtual void Stop()  = 0;

    [[nodiscard]] RobotSizeVector<robot_id_type> GetKnownIds() const
    {
        return {this->known_ids_.begin(), this->known_ids_.end()};
    }

    size_t InsertKnownIds(const RobotSizeVector<robot_id_type> &ids)
    {
        auto size_before = this->known_ids_.size();
        std::copy(ids.begin(), ids.end(), std::inserter(this->known_ids_, this->known_ids_.end()));
        return this->known_ids_.size() - size_before;
    }

private:
    known_ids_type known_ids_;
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
                    HostSizeString robot_host, const uint16_t &robot_knowledge_exchange_port,
                    const uint16_t &robot_knowledge_request_port)
        : BaseRobotCommsModel(robot_id, manager_host, manager_port, robot_host, robot_knowledge_exchange_port,
                              robot_knowledge_request_port)
    {
    }

    ~RobotCommsModel() { this->Stop(); }

    void Start() override
    {
        this->knowledge_server_ = new (this->knowledge_server_buffer_.data()) // NOLINT(cppcoreguidelines-owning-memory)
            T(std::reinterpret_pointer_cast<BaseRobotCommsModel>(this->shared_from_this()));
        this->knowledge_server_->Start();

        this->knowledge_clients_.clear();

        this->running_ = true;

        auto cfg        = esp_pthread_get_default_config();
        cfg.pin_to_core = CORE_1;
        cfg.stack_size  = 8192;
        cfg.thread_name = "robot_comms_exchange_heartbeats";
        ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg));
        this->heartbeat_thread_ = std::thread(&RobotCommsModel::LaunchExchangeHeartbeats, this);

        this->knowledge_request_socket_ = std::make_shared<asio::ip::udp::socket>(io_context_);
        this->knowledge_request_socket_->open(asio::ip::udp::v4());
        auto address         = asio::ip::make_address_v4(this->robot_host.c_str());
        auto client_endpoint = asio::ip::udp::endpoint(address, this->robot_knowledge_request_port);
        ESP_LOGI(TAG, "UDPKnowledgeServer - (%s:%hu)", client_endpoint.address().to_string().c_str(),
                 client_endpoint.port());
        this->knowledge_request_socket_->bind(client_endpoint);

        cfg             = esp_pthread_get_default_config();
        cfg.pin_to_core = CORE_1;
        cfg.stack_size  = 8192;
        cfg.thread_name = "robot_comms_knowledge_requests";
        ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg));
        this->knowledge_request_thread_ = std::thread(&RobotCommsModel::LaunchHandleKnowledgeRequests, this);
    }

    void Stop() override
    {
        this->running_ = false;
        if (this->heartbeat_thread_.joinable()) { this->heartbeat_thread_.join(); }
        if (this->knowledge_request_thread_.joinable()) { this->knowledge_request_thread_.join(); }
        if (this->knowledge_server_)
        {
            this->knowledge_server_->Stop();
            this->knowledge_server_->~T();
        }
        for (auto &[_, client] : this->knowledge_clients_)
        {
            client.Stop();
        }
        this->knowledge_clients_.clear();
    }

private:
    void LaunchExchangeHeartbeats()
    {
#if ENABLE_TRY_CATCH
        try
        {
#endif
            this->ExchangeHeartbeats();
#if ENABLE_TRY_CATCH
        } catch (const std::exception &e)
        {
            ESP_LOGE(TAG, "Error: %s", e.what());
            throw e;
        }
#endif
    }

    /**
     * @brief Exchange heartbeats with the manager, receiving a list of neighbours and connecting to them if necessary.
     *
     * ~2048 byte stack size
     *
     */
    void ExchangeHeartbeats()
    {
        auto heartbeat_client = asio::ip::udp::socket(io_context_);
        heartbeat_client.open(asio::ip::udp::v4());

        auto address = asio::ip::make_address_v4(this->manager_host.c_str());

        auto manager_endpoint = asio::ip::udp::endpoint(address, this->manager_port);

        while (this->running_)
        {
            ESP_LOGD(RobotCommsModel::TAG, "Sending heartbeat to %s:%hu", this->manager_host.c_str(),
                     this->manager_port);

            auto packet     = EpuckHeartbeatPacket();
            packet.robot_id = this->robot_id;
            strncpy(packet.robot_host.data(), this->robot_host.c_str(), MAX_HOST_LEN);
            packet.robot_port = this->robot_knowledge_exchange_port;

            ESP_LOGI(TAG, "(%s:%hu)", manager_endpoint.address().to_string().c_str(), manager_endpoint.port());
            std::array<uint8_t, sizeof(EpuckHeartbeatPacket)> packed_packet = packet.pack();
            heartbeat_client.send_to(asio::buffer(packed_packet, sizeof(EpuckHeartbeatPacket)), manager_endpoint);

            struct pollfd pfd = {heartbeat_client.native_handle(), POLLIN, 0};
            int retval        = poll(&pfd, 1, 1000);
            if (retval == 0)
            { // timeout
                ESP_LOGW(TAG, "Timeout waiting for response from " ROBOT_ID_TYPE_FMT " (%s:%hu)", this->robot_id,
                         this->manager_host.c_str(), this->manager_port);
                continue;
            }
            if (retval < 0)
            {
                ESP_LOGE(TAG, "poll %s", "error");
                perror("poll");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }

            auto response_buffer = EpuckHeartbeatResponsePacket().pack();

            ESP_LOGI(TAG, "Receiving heartbeat response%s", "");

            size_t bytes_received = 0;
            size_t expected_bytes = sizeof(EpuckHeartbeatResponsePacket);
            while (bytes_received < expected_bytes)
            {
                bytes_received +=
                    heartbeat_client.receive_from(asio::buffer(response_buffer.data() + bytes_received,
                                                               sizeof(EpuckHeartbeatResponsePacket) - bytes_received),
                                                  manager_endpoint);

                if (bytes_received > offsetof(EpuckHeartbeatResponsePacket, num_neighbours))
                {
                    auto num_neighbours = response_buffer[offsetof(EpuckHeartbeatResponsePacket, num_neighbours)];
                    expected_bytes      = offsetof(EpuckHeartbeatResponsePacket, neighbours)
                                     + num_neighbours * sizeof(EpuckNeighbourPacket);
                }
            }

            ESP_LOGD(TAG, "Received heartbeat response%s", "");
            // esp_core_dump_to_uart();

            auto response = EpuckHeartbeatResponsePacket::unpack(response_buffer.data());

            for (const auto *it = response.neighbours.begin();
                 it - response.neighbours.begin() < response.num_neighbours; it++)
            {
                const auto neighbour = *it;
                ESP_LOGD(TAG, "Received neighbour: " ROBOT_ID_TYPE_FMT " (%s:%hu) at distance %f", neighbour.robot_id,
                         neighbour.host.data(), neighbour.port, neighbour.dist);
                this->InsertKnownIds({neighbour.robot_id});
            }

            // Connect to new robots that are listed in the response if they have a lower ID
            for (const auto *it = response.neighbours.begin();
                 it - response.neighbours.begin() < response.num_neighbours; it++)
            {
                const auto neighbour = *it;
                // Only connect to robots with lower IDs that are not already connected
                if (this->knowledge_clients_.find(neighbour.robot_id) != this->knowledge_clients_.end()
                    || neighbour.robot_id >= this->robot_id)
                {
                    continue;
                }

                ESP_LOGI(TAG, "Starting thread for neighbour " ROBOT_ID_TYPE_FMT " (%s:%hu)", neighbour.robot_id,
                         neighbour.host.data(), neighbour.port);

                U client(
                    neighbour,
                    [this, neighbour]() {
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

                ESP_LOGI(TAG, "Stopping thread for neighbour " ROBOT_ID_TYPE_FMT, neighbour_id);

                this->knowledge_clients_[neighbour_id].Stop();
                this->knowledge_clients_.erase(neighbour_id);
            }

            // Sleep for 1 second
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    void LaunchHandleKnowledgeRequests()
    {
#if ENABLE_TRY_CATCH
        try
        {
#endif
            ESP_LOGI(TAG, "Starting knowledge request connection on %s:%hu", this->robot_host.c_str(),
                     this->robot_knowledge_request_port);
            while (this->running_)
            {

                struct timeval tv = {1, 0};
                fd_set readfds;
                FD_ZERO(&readfds);
                FD_SET(this->knowledge_request_socket_->native_handle(), &readfds);
                int fds_ready =
                    select(this->knowledge_request_socket_->native_handle() + 1, &readfds, nullptr, nullptr, &tv);
                if (fds_ready == 0)
                { // timeout
                    ESP_LOGD(TAG, "Knowledge Request server timeout, no data received%s", "");
                    continue;
                }
                if (fds_ready < 0)
                {
                    ESP_LOGE(TAG, "Error receiving data - %s", "select");
                    perror("select");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    continue;
                }

                asio::ip::udp::endpoint client;
                std::array<uint8_t, sizeof(EpuckKnowledgePacket)> data{};

                size_t bytes_received = 0;
                size_t expected_bytes = sizeof(EpuckKnowledgePacket);
                while (bytes_received < expected_bytes)
                {
                    auto received = this->knowledge_request_socket_->receive_from(
                        asio::buffer(data.data() + bytes_received, sizeof(EpuckKnowledgePacket) - bytes_received),
                        client);
                    if (received < 1)
                    {
                        ESP_LOGW(TAG, "Knowledge request client (%s:%hu) disconnected",
                                 client.address().to_string().c_str(), client.port());
                        break;
                    }
                    bytes_received += received;
                    if (bytes_received > offsetof(EpuckKnowledgePacket, N))
                    {
                        expected_bytes =
                            offsetof(EpuckKnowledgePacket, known_ids) + data[offsetof(EpuckKnowledgePacket, N)];
                    }
                }

                if (bytes_received != expected_bytes)
                {
                    ESP_LOGW(TAG, "Received %zu bytes, expected %zu bytes", bytes_received, expected_bytes);
                    continue;
                }

                this->HandleKnowledgeRequests(client, data);
            }
#if ENABLE_TRY_CATCH
        } catch (const std::exception &e)
        {
            ESP_LOGE(TAG, "Error: %s", e.what());
            throw e;
        }
#endif
    }

    void HandleKnowledgeRequests(const asio::ip::udp::endpoint &client,
                                 const std::array<uint8_t, sizeof(EpuckKnowledgePacket)> &data)
    {
        auto request = EpuckKnowledgePacket::unpack(data.data());

        ESP_LOGD(TAG, "Received knowledge request from %s:%hu", client.address().to_string().c_str(), client.port());

        auto known_ids = this->GetKnownIds();

        auto knowledge     = EpuckKnowledgePacket();
        knowledge.robot_id = this->robot_id;
        knowledge.N        = std::distance(known_ids.begin(), known_ids.end());
        std::copy(known_ids.begin(), known_ids.end(), knowledge.known_ids.begin());

        this->knowledge_request_socket_->send_to(asio::buffer(knowledge.pack(), sizeof(EpuckKnowledgePacket)), client);

        ESP_LOGD(TAG, "Sent knowledge to %s:%hu - %s", client.address().to_string().c_str(), client.port(),
                 known_ids_list_to_string(knowledge.known_ids, knowledge.N).data());
    }

    asio::io_context io_context_;
    alignas(T) std::array<uint8_t, sizeof(T)> knowledge_server_buffer_ = {0};
    T *knowledge_server_;
    RobotSizeMap<robot_id_type, U> knowledge_clients_;

    bool running_ = false;

    std::thread heartbeat_thread_;

    std::shared_ptr<asio::ip::udp::socket> knowledge_request_socket_ = nullptr;
    std::thread knowledge_request_thread_;

    static constexpr char TAG[] = "RobotCommsModel";
};
