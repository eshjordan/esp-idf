#pragma once

#include "EpuckPackets.hpp"
#include "NetworkFactory.hpp"
#include "RobotCommsModel.hpp"
#include "types.hpp"
#include <algorithm>
#include <array>
#include <asio/io_context.hpp>
#include <asio/ip/address_v6.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sys/select.h>
#include <thread>
#include <utility>

template <class T>
using robot_id_list_allocator =
    StaticAllocator<T, ((sizeof("65535") - 1) * MAX_ROBOTS) + ((sizeof(", ") - 1) * (MAX_ROBOTS - 1)) + sizeof("")>;

// DECLARE_STATIC_ALLOCATOR(RobotIdListAllocator,
//                          ((sizeof("255") - 1) * MAX_ROBOTS) + ((sizeof(", ") - 1) * (MAX_ROBOTS -
//                          1)) + sizeof(""));

class UDPKnowledgeClient;

class UDPKnowledgeServer : public BaseKnowledgeServer
{
public:
    UDPKnowledgeServer() = default;
    template <class T>
    explicit UDPKnowledgeServer(std::shared_ptr<T> robot_model, std::shared_ptr<INetworkFactory> network_factory)
        : BaseKnowledgeServer(std::move(robot_model), std::move(network_factory))
    {
        _io_context = _network_factory->create_io_context();
    }

    // Move constructor
    UDPKnowledgeServer(UDPKnowledgeServer &&other) noexcept
        : _knowledge_server_socket(std::move(other._knowledge_server_socket)), _running(other._running)
    {
        _robot_model     = std::move(other._robot_model);
        _network_factory = std::move(other._network_factory);

        this->_receive_knowledge_thread.swap(other._receive_knowledge_thread);
    }

    // Move assignment
    UDPKnowledgeServer &operator=(UDPKnowledgeServer &&other) noexcept
    {
        _robot_model             = std::move(other._robot_model);
        _network_factory         = std::move(other._network_factory);
        _knowledge_server_socket = std::move(other._knowledge_server_socket);

        this->_receive_knowledge_thread.swap(other._receive_knowledge_thread);
        this->_running = other._running;
        return *this;
    }

    // Copy constructor
    UDPKnowledgeServer(const UDPKnowledgeServer &other)
        : BaseKnowledgeServer(other._robot_model, other._network_factory),
          _knowledge_server_socket(other._knowledge_server_socket), _running(other._running)
    {
        _robot_model     = other._robot_model;
        _network_factory = other._network_factory;

        // this->thread_.swap(other.thread_);
    }

    // Copy assignment
    UDPKnowledgeServer &operator=(const UDPKnowledgeServer &other)
    {
        _robot_model             = other._robot_model;
        _network_factory         = other._network_factory;
        _knowledge_server_socket = other._knowledge_server_socket;
        _running                 = other._running;
        return *this;
    }

    // Destructor
    ~UDPKnowledgeServer() { this->stop(); }

    void start() override
    {
        this->_running                 = true;
        this->_knowledge_server_socket = _network_factory->create_udp_socket(*_io_context);
        this->_knowledge_server_socket->open();
        auto address = asio::ip::make_address_v4(this->_robot_model->robot_knowledge_host.c_str());
        auto client_endpoint =
            _network_factory->create_udp_endpoint(address, this->_robot_model->robot_knowledge_exchange_port);
        this->_knowledge_server_socket->bind(*client_endpoint);

        auto cfg        = esp_pthread_get_default_config();
        cfg.pin_to_core = CORE_1;
        cfg.stack_size  = 8192;
        cfg.thread_name = "udp_server_start_receive";
        ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg));
        this->_receive_knowledge_thread = std::thread(&UDPKnowledgeServer::launch_receive_knowledge, this);
    }

    void stop() noexcept override
    {
        this->_running = false;
        if (this->_receive_knowledge_thread.joinable()) { this->_receive_knowledge_thread.join(); }
        if (nullptr != this->_knowledge_server_socket) { this->_knowledge_server_socket->close(); }
        this->_knowledge_server_socket = nullptr;
    }

private:
    void launch_receive_knowledge()
    {
#if ENABLE_TRY_CATCH
        try
        {
#endif
            this->receive_knowledge();
#if ENABLE_TRY_CATCH
        } catch (const std::exception &e)
        {
            ESP_LOGE(TAG, "Error: %s", e.what());
            throw e;
        }
#endif
    }

    void receive_knowledge()
    {
        ESP_LOGI(TAG, "Starting knowledge server at %s:%hu", this->_robot_model->robot_knowledge_host.c_str(),
                 this->_robot_model->robot_knowledge_exchange_port);
        while (this->_running)
        {
            struct timeval tv = {1, 0};
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(this->_knowledge_server_socket->get_native_socket().native_handle(), &readfds);
            int fds_ready = select(this->_knowledge_server_socket->get_native_socket().native_handle() + 1, &readfds,
                                   nullptr, nullptr, &tv);
            if (fds_ready == 0)
            { // timeout
                ESP_LOGD(TAG, "Timeout, no data received%s", "");
                continue;
            }
            if (fds_ready < 0)
            {
                ESP_LOGE(TAG, "Error receiving data - %s", "select");
                perror("select");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }

            auto client = _network_factory->create_udp_endpoint();
            std::array<uint8_t, sizeof(EpuckKnowledgePacket)> data{};

            size_t bytes_received = 0;
            size_t expected_bytes = sizeof(EpuckKnowledgePacket);
            while (bytes_received < expected_bytes)
            {
                auto received = this->_knowledge_server_socket->receive_from(
                    asio::buffer(data.data() + bytes_received, sizeof(EpuckKnowledgePacket) - bytes_received), *client);
                if (received < 1)
                {
                    ESP_LOGW(TAG, "(%s:%hu) disconnected", client->address().to_string().c_str(), client->port());
                    break;
                }
                bytes_received += received;
                // if (bytes_received >= offsetof(EpuckKnowledgePacket, N) + sizeof(EpuckKnowledgePacket::N))
                // {
                //     const auto n =
                //         *static_cast<decltype(EpuckKnowledgePacket::N) *>(&data[offsetof(EpuckKnowledgePacket, N)]);
                //     expected_bytes = offsetof(EpuckKnowledgePacket, known_ids) + (n * sizeof(EpuckKnowledgeRecord));
                // }
            }

            if (bytes_received != expected_bytes)
            {
                ESP_LOGW(TAG, "Received %zu bytes, expected %zu bytes", bytes_received, expected_bytes);
                continue;
            }

            handle_knowledge_packet(*client, data);
        }
    }

    void handle_knowledge_packet(IUDPEndpoint &client, const std::array<uint8_t, sizeof(EpuckKnowledgePacket)> &data)
    {
        auto request = EpuckKnowledgePacket::unpack(data.data());

        ESP_LOGD(
            TAG, "Received knowledge from " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", request.robot_id,
            client.address().to_string().c_str(), client.port(),
            known_ids_to_string(request.known_ids.cbegin(), request.known_ids.cbegin() + request.num_known_ids).data());

        auto known_ids_before = robot_size_set<BaseRobotCommsModel::known_id_record_type>(
            this->_robot_model->known_ids_begin(), this->_robot_model->known_ids_end());

        auto num_inserted = this->_robot_model->insert_known_ids(request.known_ids.cbegin(),
                                                                 request.known_ids.cbegin() + request.num_known_ids);

        if (num_inserted > 0)
        {
            robot_size_set<BaseRobotCommsModel::known_id_record_type> new_ids;
            std::set_difference(this->_robot_model->known_ids_begin(), this->_robot_model->known_ids_end(),
                                known_ids_before.cbegin(), known_ids_before.cend(),
                                std::inserter(new_ids, new_ids.end()));

            ESP_LOGI(TAG, "Knowledge server received new IDs from " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", request.robot_id,
                     client.address().to_string().c_str(), client.port(),
                     known_ids_to_string(new_ids.cbegin(), new_ids.cend()).data());
        }

        auto knowledge = this->_robot_model->create_knowledge_packet();

        _knowledge_server_socket->send_to(asio::buffer(knowledge.pack(), sizeof(EpuckKnowledgePacket)), client);

        ESP_LOGD(
            TAG, "Sent knowledge to " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", request.robot_id,
            client.address().to_string().c_str(), client.port(),
            known_ids_to_string(knowledge.known_ids.cbegin(), knowledge.known_ids.cbegin() + knowledge.num_known_ids)
                .data());
    }

    bool _running = false;
    std::thread _receive_knowledge_thread;
    std::shared_ptr<IIoContext> _io_context;
    std::shared_ptr<IUdpSocket> _knowledge_server_socket = nullptr;
    constexpr static const char *const TAG = "UDPKnowledgeServer"; // NOLINT(cppcoreguidelines-avoid-c-arrays)
};

class UDPKnowledgeClient : public BaseKnowledgeClient
{
public:
    UDPKnowledgeClient() = default;
    UDPKnowledgeClient(EpuckNeighbourPacket neighbour, std::function<bool()> running,
                       std::shared_ptr<BaseRobotCommsModel> robot_model,
                       std::shared_ptr<INetworkFactory> network_factory)
        : BaseKnowledgeClient(neighbour, std::move(running), std::move(robot_model), std::move(network_factory))
    {
        _io_context = _network_factory->create_io_context();
    };

    // Move constructor
    UDPKnowledgeClient(UDPKnowledgeClient &&other) noexcept
        : _io_context(std::move(other._io_context)),
          _knowledge_client_socket(std::move(other._knowledge_client_socket)), _stopping(other._stopping)
    {
        _neighbour       = other._neighbour;
        _running         = std::move(other._running);
        _robot_model     = std::move(other._robot_model);
        _network_factory = std::move(other._network_factory);

        this->_send_knowledge_thread.swap(other._send_knowledge_thread);
    }

    // Move assignment
    UDPKnowledgeClient &operator=(UDPKnowledgeClient &&other) noexcept
    {
        _io_context              = std::move(other._io_context);
        _neighbour               = other._neighbour;
        _running                 = std::move(other._running);
        _robot_model             = std::move(other._robot_model);
        _network_factory         = std::move(other._network_factory);
        _knowledge_client_socket = std::move(other._knowledge_client_socket);

        this->_send_knowledge_thread.swap(other._send_knowledge_thread);
        _stopping = other._stopping;

        return *this;
    }

    // Copy constructor
    UDPKnowledgeClient(const UDPKnowledgeClient &other)
        : BaseKnowledgeClient(other._neighbour, other._running, other._robot_model, other._network_factory),
          _io_context(other._io_context), _knowledge_client_socket(other._knowledge_client_socket),
          _stopping(other._stopping)
    {
    }

    // Copy assignment
    UDPKnowledgeClient &operator=(const UDPKnowledgeClient &other)
    {
        _io_context              = other._io_context;
        _neighbour               = other._neighbour;
        _running                 = other._running;
        _robot_model             = other._robot_model;
        _network_factory         = other._network_factory;
        _knowledge_client_socket = other._knowledge_client_socket;
        _stopping                = other._stopping;

        return *this;
    }

    // Destructor
    ~UDPKnowledgeClient() { this->stop(); }

    void start() override
    {
        this->_stopping                = false;
        this->_knowledge_client_socket = _network_factory->create_udp_socket(*_io_context);
        this->_knowledge_client_socket->open();

        auto cfg        = esp_pthread_get_default_config();
        cfg.stack_size  = 8192;
        cfg.pin_to_core = CORE_1;

        std::array<char, 64> thread_name = {0};
        snprintf(thread_name.data(), sizeof(thread_name), "udp_client_" ROBOT_ID_TYPE_FMT "_send_knowledge",
                 this->_neighbour.robot_id);
        cfg.thread_name = thread_name.data();

        ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg));
        this->_send_knowledge_thread = std::thread(&UDPKnowledgeClient::launch_send_knowledge, this);
    }

    void stop() noexcept override
    {
        this->_stopping = true;
        if (this->_send_knowledge_thread.joinable()) { this->_send_knowledge_thread.join(); }
        if (nullptr != this->_knowledge_client_socket) { this->_knowledge_client_socket->close(); }
        this->_knowledge_client_socket = nullptr;
    }

private:
    void launch_send_knowledge()
    {
#if ENABLE_TRY_CATCH
        try
        {
#endif
            this->send_knowledge();
#if ENABLE_TRY_CATCH
        } catch (const std::exception &e)
        {
            ESP_LOGE(TAG, "Error: %s", e.what());
            throw e;
        }
#endif
    }

    void send_knowledge()
    {
        ESP_LOGI(TAG, "Starting knowledge client to " ROBOT_ID_TYPE_FMT " (%s:%hu)", this->_neighbour.robot_id,
                 this->_neighbour.host.data(), this->_neighbour.port);

        auto server = _network_factory->create_udp_endpoint(asio::ip::make_address_v4(this->_neighbour.host.data()),
                                                            this->_neighbour.port);

        while (this->_running() && !this->_stopping && this->_knowledge_client_socket
               && this->_knowledge_client_socket->is_open())
        {
            auto knowledge = this->_robot_model->create_knowledge_packet();

            _knowledge_client_socket->send_to(asio::buffer(knowledge.pack(), sizeof(EpuckKnowledgePacket)), *server);

            ESP_LOGD(TAG, "Sent knowledge to " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", this->_neighbour.robot_id,
                     this->_neighbour.host.data(), this->_neighbour.port,
                     known_ids_to_string(knowledge.known_ids.cbegin(),
                                         knowledge.known_ids.cbegin() + knowledge.num_known_ids)
                         .data());

            struct pollfd pfd = {this->_knowledge_client_socket->get_native_socket().native_handle(), POLLIN, 0};
            int retval        = poll(&pfd, 1, 1000);
            if (retval == 0)
            { // timeout
                ESP_LOGW(TAG, "Timeout waiting for response from " ROBOT_ID_TYPE_FMT " (%s:%hu)",
                         this->_neighbour.robot_id, this->_neighbour.host.data(), this->_neighbour.port);
                continue;
            }
            if (retval < 0)
            {
                ESP_LOGE(TAG, "poll %s", "error");
                perror("poll");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }

            std::array<uint8_t, sizeof(EpuckKnowledgePacket)> data{};

            size_t bytes_received = 0;
            size_t expected_bytes = sizeof(EpuckKnowledgePacket);
            while (bytes_received < expected_bytes)
            {
                auto received = this->_knowledge_client_socket->receive_from(
                    asio::buffer(data.data() + bytes_received, sizeof(EpuckKnowledgePacket) - bytes_received), *server);
                if (received < 1)
                {
                    ESP_LOGW(TAG, "(%s:%hu) disconnected", this->_neighbour.host.data(), this->_neighbour.port);
                    break;
                }
                bytes_received += received;
                if (bytes_received > offsetof(EpuckKnowledgePacket, num_known_ids))
                {
                    expected_bytes = offsetof(EpuckKnowledgePacket, known_ids)
                                     + data[offsetof(EpuckKnowledgePacket, num_known_ids)]
                                           * sizeof(EpuckKnowledgePacket::known_ids[0]);
                }
            }

            auto response = EpuckKnowledgePacket::unpack(data.data());

            ESP_LOGD(
                TAG, "Received knowledge from " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", response.robot_id,
                this->_neighbour.host.data(), this->_neighbour.port,
                known_ids_to_string(response.known_ids.cbegin(), response.known_ids.cbegin() + response.num_known_ids)
                    .data());

            auto known_ids_before = robot_size_set<BaseRobotCommsModel::known_id_record_type>(
                this->_robot_model->known_ids_begin(), this->_robot_model->known_ids_end());

            auto num_inserted = this->_robot_model->insert_known_ids(
                response.known_ids.cbegin(), response.known_ids.cbegin() + response.num_known_ids);

            if (num_inserted > 0)
            {
                robot_size_set<BaseRobotCommsModel::known_id_record_type> new_ids;
                std::set_difference(this->_robot_model->known_ids_begin(), this->_robot_model->known_ids_end(),
                                    known_ids_before.cbegin(), known_ids_before.cend(),
                                    std::inserter(new_ids, new_ids.begin()));

                ESP_LOGI(TAG, "Knowledge client received new IDs from " ROBOT_ID_TYPE_FMT " (%s:%hu): %s",
                         response.robot_id, this->_neighbour.host.data(), this->_neighbour.port,
                         known_ids_to_string(new_ids.cbegin(), new_ids.cend()).data());
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        ESP_LOGI(TAG, "Stopping knowledge client to " ROBOT_ID_TYPE_FMT " (%s:%hu)", this->_neighbour.robot_id,
                 this->_neighbour.host.data(), this->_neighbour.port);
    }

    bool _stopping = false;
    std::thread _send_knowledge_thread;
    std::shared_ptr<IIoContext> _io_context;
    std::shared_ptr<IUdpSocket> _knowledge_client_socket = nullptr;
    static constexpr const char *const TAG               = "UDPKnowledgeClient";
};
