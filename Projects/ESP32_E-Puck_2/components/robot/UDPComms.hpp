#pragma once

#include "EpuckPackets.hpp"
#include "RobotCommsModel.hpp"
#include "types.hpp"
#include <algorithm>
#include <array>
#include <asio/io_context.hpp>
#include <asio/ip/address_v6.hpp>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sstream>
#include <sys/select.h>
#include <thread>
#include <utility>

template <class T> using RobotIdListAllocator =
    static_allocator<T, ((sizeof("255") - 1) * MAX_ROBOTS) + ((sizeof(", ") - 1) * (MAX_ROBOTS - 1)) + sizeof("")>;
// DECLARE_STATIC_ALLOCATOR(RobotIdListAllocator,
//                          ((sizeof("255") - 1) * MAX_ROBOTS) + ((sizeof(", ") - 1) * (MAX_ROBOTS - 1)) + sizeof(""));

static inline auto known_ids_set_to_string(const RobotSizeSet<robot_id_type> &known_ids)
{
    std::array<char, ((sizeof("255") - 1) * MAX_ROBOTS) + ((sizeof(", ") - 1) * (MAX_ROBOTS - 1)) + sizeof("")> output =
        {0};
    if (known_ids.empty())
    {
        snprintf(output.data(), sizeof("{}"), "{}");
        return output;
    }

    snprintf(output.data(), sizeof(output), "%hhu", (*known_ids.begin()));
    for (auto it = known_ids.begin(); ++it != known_ids.end();)
    {
        std::array<char, sizeof(", 255")> buf = {0};
        snprintf(buf.data(), sizeof(buf), ", %hhu", (*it));
        strncat(output.data(), buf.data(), sizeof(buf));
    }
    return output;
}

static inline auto known_ids_list_to_string(const std::array<robot_id_type, MAX_ROBOTS> &known_ids, uint8_t N)
{
    std::array<char, ((sizeof("255") - 1) * MAX_ROBOTS) + ((sizeof(", ") - 1) * (MAX_ROBOTS - 1)) + sizeof("")> output =
        {0};
    if (known_ids.empty())
    {
        snprintf(output.data(), sizeof("{}"), "{}");
        return output;
    }

    snprintf(output.data(), sizeof(output), "%hhu", known_ids[0]);
    for (size_t i = 1; i < N; i++)
    {
        std::array<char, sizeof(", 255")> buf = {0};
        snprintf(buf.data(), sizeof(buf), ", %hhu", known_ids[i]);
        strncat(output.data(), buf.data(), sizeof(buf));
    }
    return output;
}

class UDPKnowledgeClient;

class UDPKnowledgeServer : public BaseKnowledgeServer
{
public:
    UDPKnowledgeServer() = default;
    template <class T> explicit UDPKnowledgeServer(std::shared_ptr<T> robot_model) : BaseKnowledgeServer(robot_model) {}

    // Move constructor
    UDPKnowledgeServer(UDPKnowledgeServer &&other) noexcept
    {
        robot_model = std::move(other.robot_model);
        if (nullptr != other.socket_)
        {
            this->socket_ = other.socket_;
        } else
        {
            this->socket_ = nullptr;
        }
        this->thread_.swap(other.thread_);
        this->running_ = other.running_;
    }

    // Move assignment
    UDPKnowledgeServer &operator=(UDPKnowledgeServer &&other) noexcept
    {
        robot_model = std::move(other.robot_model);
        if (nullptr != other.socket_)
        {
            this->socket_ = other.socket_;
        } else
        {
            this->socket_ = nullptr;
        }
        this->thread_.swap(other.thread_);
        this->running_ = other.running_;
        return *this;
    }

    // Copy constructor
    UDPKnowledgeServer(const UDPKnowledgeServer &other) : BaseKnowledgeServer(other.robot_model)
    {
        robot_model = other.robot_model;
        if (nullptr != other.socket_)
        {
            this->socket_ = other.socket_;
        } else
        {
            this->socket_ = nullptr;
        }
        // this->thread_.swap(other.thread_);
        this->running_ = other.running_;
    }

    // Copy assignment
    UDPKnowledgeServer &operator=(const UDPKnowledgeServer &other)
    {
        robot_model = other.robot_model;
        if (nullptr != other.socket_)
        {
            this->socket_ = other.socket_;
        } else
        {
            this->socket_ = nullptr;
        }
        this->running_ = other.running_;
        return *this;
    }

    // Destructor
    ~UDPKnowledgeServer() { this->Stop(); }

    void Start() override
    {
        this->running_ = true;
        this->socket_  = std::make_shared<asio::ip::udp::socket>(io_context_);
        this->socket_->open(asio::ip::udp::v4());
        auto address         = asio::ip::make_address_v4(this->robot_model->robot_host.c_str());
        auto client_endpoint = asio::ip::udp::endpoint(address, this->robot_model->robot_port);
        ESP_LOGI(TAG, "UDPKnowledgeServer - (%s:%hu)", client_endpoint.address().to_string().c_str(),
                 client_endpoint.port());
        this->socket_->bind(client_endpoint);

        auto cfg        = esp_pthread_get_default_config();
        cfg.pin_to_core = CORE_1;
        cfg.thread_name = "udp_server_start_receive";
        ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg));
        this->thread_ = std::thread(&UDPKnowledgeServer::LaunchStartReceive, this);
    }

    void Stop() override
    {
        this->running_ = false;
        if (this->thread_.joinable()) { this->thread_.join(); }
        if (nullptr != this->socket_) { this->socket_->close(); }
        this->socket_ = nullptr;
    }

private:
    asio::io_context io_context_;
    std::shared_ptr<asio::ip::udp::socket> socket_ = nullptr;
    std::thread thread_;
    bool running_               = false;
    static constexpr char TAG[] = "UDPKnowledgeServer";

    void LaunchStartReceive()
    {
#if ENABLE_TRY_CATCH
        try
        {
#endif
            this->StartReceive();
#if ENABLE_TRY_CATCH
        } catch (const std::exception &e)
        {
            ESP_LOGE(TAG, "Error: %s", e.what());
            throw e;
        }
#endif
    }

    void StartReceive()
    {
        ESP_LOGI(TAG, "Starting knowledge connection on %s:%hu", this->robot_model->robot_host.c_str(),
                 this->robot_model->robot_port);
        while (this->running_)
        {
            struct timeval tv = {1, 0};
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(this->socket_->native_handle(), &readfds);
            int fds_ready = select(this->socket_->native_handle() + 1, &readfds, nullptr, nullptr, &tv);
            if (fds_ready == 0)
            { // timeout
                ESP_LOGD(TAG, "Timeout, no data received%s", "");
                continue;
            }
            if (fds_ready < 0)
            {
                ESP_LOGE(TAG, "Error receiving data%s", "select");
                perror("select");
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }

            // struct pollfd pfd = {this->socket_->native_handle(), POLLIN, 0};
            // int retval        = poll(&pfd, 1, 1000);
            // if (retval == 0)
            // { // timeout
            //     ESP_LOGD(TAG, "Timeout, no data received%s", "");
            //     continue;
            // }
            // if (retval < 0)
            // {
            //     ESP_LOGE(TAG, "Error receiving data%s", "poll");
            //     perror("poll");
            //     vTaskDelay(1000 / portTICK_PERIOD_MS);
            //     continue;
            // }

            asio::ip::udp::endpoint client;
            std::array<uint8_t, sizeof(EpuckKnowledgePacket)> data{};

            size_t bytes_received = 0;
            size_t expected_bytes = sizeof(EpuckKnowledgePacket);
            while (bytes_received < expected_bytes)
            {
                auto received = this->socket_->receive_from(
                    asio::buffer(data.data() + bytes_received, sizeof(EpuckKnowledgePacket) - bytes_received), client);
                if (received < 1)
                {
                    ESP_LOGW(TAG, "(%s:%hu) disconnected", client.address().to_string().c_str(), client.port());
                    break;
                }
                bytes_received += received;
                if (bytes_received > offsetof(EpuckKnowledgePacket, N))
                {
                    expected_bytes =
                        offsetof(EpuckKnowledgePacket, known_ids) + data[offsetof(EpuckKnowledgePacket, N)];
                }
            }

            HandleReceive(client, data);
        }
    }

    void HandleReceive(const asio::ip::udp::endpoint &client,
                       const std::array<uint8_t, sizeof(EpuckKnowledgePacket)> &data)
    {
        auto request = EpuckKnowledgePacket::unpack(data.data());

        auto known_ids_before = this->robot_model->GetKnownIds();

        auto num_inserted =
            this->robot_model->InsertKnownIds({request.known_ids.begin(), request.known_ids.begin() + request.N});

        auto known_ids_after = this->robot_model->GetKnownIds();

        if (num_inserted > 0)
        {
            RobotSizeSet<robot_id_type> new_ids;
            std::set_difference(request.known_ids.begin(), request.known_ids.begin() + request.N,
                                known_ids_before.begin(), known_ids_before.end(),
                                std::inserter(new_ids, new_ids.begin()));

            ESP_LOGI(TAG, "Received new IDs from " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", request.robot_id,
                     client.address().to_string().c_str(), client.port(), known_ids_set_to_string(new_ids).data());
        }

        ESP_LOGD(TAG, "Received knowledge from " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", request.robot_id,
                 client.address().to_string().c_str(), client.port(),
                 known_ids_list_to_string(request.known_ids, request.N).data());

        auto knowledge     = EpuckKnowledgePacket();
        knowledge.robot_id = this->robot_model->robot_id;
        knowledge.N        = std::distance(known_ids_after.begin(), known_ids_after.end());
        std::copy(known_ids_after.begin(), known_ids_after.end(), knowledge.known_ids.begin());

        socket_->send_to(asio::buffer(knowledge.pack(), sizeof(EpuckKnowledgePacket)), client);

        ESP_LOGD(TAG, "Sent knowledge to " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", request.robot_id,
                 client.address().to_string().c_str(), client.port(),
                 known_ids_list_to_string(knowledge.known_ids, knowledge.N).data());
    }
};

class UDPKnowledgeClient : public BaseKnowledgeClient
{
public:
    UDPKnowledgeClient() = default;
    UDPKnowledgeClient(EpuckNeighbourPacket neighbour, std::function<bool()> running,
                       std::shared_ptr<BaseRobotCommsModel> robot_model)
        : BaseKnowledgeClient(std::move(neighbour), std::move(running), std::move(robot_model)){};

    // Move constructor
    UDPKnowledgeClient(UDPKnowledgeClient &&other) noexcept : stopping_(other.stopping_)
    {
        neighbour   = std::move(other.neighbour);
        running     = std::move(other.running);
        robot_model = std::move(other.robot_model);
        if (nullptr != other.client_)
        {
            this->client_ = other.client_;
        } else
        {
            this->client_ = nullptr;
        }
        this->thread_.swap(other.thread_);
    }

    // Move assignment
    UDPKnowledgeClient &operator=(UDPKnowledgeClient &&other) noexcept
    {
        neighbour   = std::move(other.neighbour);
        running     = std::move(other.running);
        robot_model = std::move(other.robot_model);
        if (nullptr != other.client_)
        {
            this->client_ = other.client_;
        } else
        {
            this->client_ = nullptr;
        }
        this->thread_.swap(other.thread_);
        this->stopping_ = other.stopping_;

        return *this;
    }

    // Copy constructor
    UDPKnowledgeClient(const UDPKnowledgeClient &other)
        : BaseKnowledgeClient(other.neighbour, other.running, other.robot_model), stopping_(other.stopping_)
    {
        if (nullptr != other.client_)
        {
            this->client_ = other.client_;
        } else
        {
            this->client_ = nullptr;
        }
    }

    // Copy assignment
    UDPKnowledgeClient &operator=(const UDPKnowledgeClient &other)
    {
        neighbour   = other.neighbour;
        running     = other.running;
        robot_model = other.robot_model;
        if (nullptr != other.client_)
        {
            this->client_ = other.client_;
        } else
        {
            this->client_ = nullptr;
        }
        this->stopping_ = other.stopping_;

        return *this;
    }

    // Destructor
    ~UDPKnowledgeClient() { this->Stop(); }

    void Start() override
    {
        this->stopping_ = false;
        this->client_   = std::make_shared<asio::ip::udp::socket>(io_context_);
        this->client_->open(asio::ip::udp::v4());

        auto cfg             = esp_pthread_get_default_config();
        cfg.stack_size       = 4096;
        cfg.pin_to_core      = CORE_1;
        char thread_name[64] = {0};
        snprintf(thread_name, sizeof(thread_name), "udp_client_" ROBOT_ID_TYPE_FMT "_send_knowledge",
                 this->neighbour.robot_id);
        cfg.thread_name = thread_name;
        ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg));
        this->thread_ = std::thread(&UDPKnowledgeClient::LaunchSendKnowledge, this);
    }

    void Stop() override
    {
        this->stopping_ = true;
        if (this->thread_.joinable()) { this->thread_.join(); }
        if (nullptr != this->client_) { this->client_->close(); }
        this->client_ = nullptr;
    }

private:
    asio::io_context io_context_;
    std::shared_ptr<asio::ip::udp::socket> client_ = nullptr;
    std::thread thread_;
    bool stopping_              = false;
    static constexpr char TAG[] = "UDPKnowledgeClient";

    void LaunchSendKnowledge()
    {
#if ENABLE_TRY_CATCH
        try
        {
#endif
            this->SendKnowledge();
#if ENABLE_TRY_CATCH
        } catch (const std::exception &e)
        {
            ESP_LOGE(TAG, "Error: %s", e.what());
            throw e;
        }
#endif
    }

    void SendKnowledge()
    {
        ESP_LOGI(TAG, "Starting knowledge connection with " ROBOT_ID_TYPE_FMT " (%s:%hu)", this->neighbour.robot_id,
                 this->neighbour.host.data(), this->neighbour.port);

        auto server =
            asio::ip::udp::endpoint(asio::ip::make_address_v4(this->neighbour.host.data()), this->neighbour.port);

        while (this->running() && !this->stopping_)
        {
            auto knowledge     = EpuckKnowledgePacket();
            knowledge.robot_id = this->robot_model->robot_id;
            auto known_ids     = this->robot_model->GetKnownIds();
            std::copy(known_ids.begin(), known_ids.end(), knowledge.known_ids.begin());
            knowledge.N = known_ids.size();

            client_->send_to(asio::buffer(knowledge.pack(), sizeof(EpuckKnowledgePacket)), server);

            ESP_LOGD(TAG, "Sent knowledge to " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", this->neighbour.robot_id,
                     this->neighbour.host.data(), this->neighbour.port,
                     known_ids_list_to_string(knowledge.known_ids, knowledge.N).data());

            struct pollfd pfd = {this->client_->native_handle(), POLLIN, 0};
            int retval        = poll(&pfd, 1, 1000);
            if (retval == 0)
            { // timeout
                ESP_LOGW(TAG, "Timeout waiting for response from " ROBOT_ID_TYPE_FMT " (%s:%hu)",
                         this->neighbour.robot_id, this->neighbour.host.data(), this->neighbour.port);
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
                auto received = this->client_->receive_from(
                    asio::buffer(data.data() + bytes_received, sizeof(EpuckKnowledgePacket) - bytes_received), server);
                if (received < 1)
                {
                    ESP_LOGW(TAG, "(%s:%hu) disconnected", this->neighbour.host.data(), this->neighbour.port);
                    break;
                }
                bytes_received += received;
                if (bytes_received > offsetof(EpuckKnowledgePacket, N))
                {
                    expected_bytes =
                        offsetof(EpuckKnowledgePacket, known_ids) + data[offsetof(EpuckKnowledgePacket, N)];
                }
            }

            auto response = EpuckKnowledgePacket::unpack(data.data());

            const auto &known_ids_before = knowledge.known_ids;

            auto num_inserted = this->robot_model->InsertKnownIds(
                {response.known_ids.begin(), response.known_ids.begin() + response.N});

            if (num_inserted > 0)
            {
                RobotSizeSet<robot_id_type> new_ids;
                std::set_difference(response.known_ids.begin(), response.known_ids.end(), known_ids_before.begin(),
                                    known_ids_before.end(), std::inserter(new_ids, new_ids.begin()));

                ESP_LOGI(TAG, "Received new IDs from " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", response.robot_id,
                         this->neighbour.host.data(), this->neighbour.port, known_ids_set_to_string(new_ids).data());
            }

            ESP_LOGD(TAG, "Received knowledge from " ROBOT_ID_TYPE_FMT " (%s:%hu): %s", response.robot_id,
                     this->neighbour.host.data(), this->neighbour.port,
                     known_ids_list_to_string(response.known_ids, response.N).data());

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
};
