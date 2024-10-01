#pragma once

#include "EpuckPackets.hpp"
#include "RobotCommsModel.hpp"
#include "types.hpp"
#include <algorithm>
#include <array>
#include <asio/io_context.hpp>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <thread>

template <class T> using RobotIdListAllocator =
    static_allocator<T, ((sizeof("255") - 1) * MAX_ROBOTS) + ((sizeof(", ") - 1) * (MAX_ROBOTS - 1)) + sizeof("")>;

static inline auto known_ids_to_string(const RobotSizeSet<uint8_t> &known_ids)
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

class UDPKnowledgeClient;

class UDPKnowledgeServer : public BaseKnowledgeServer
{
public:
    UDPKnowledgeServer() = default;
    template <class T> explicit UDPKnowledgeServer(std::shared_ptr<T> robot_model) : BaseKnowledgeServer(robot_model) {}

    // Move constructor
    UDPKnowledgeServer(UDPKnowledgeServer &&other) noexcept : BaseKnowledgeServer(std::move(other)) {}

    // Move assignment
    UDPKnowledgeServer &operator=(UDPKnowledgeServer &&other) noexcept
    {
        BaseKnowledgeServer::operator=(std::move(other));
        return *this;
    }

    // Copy constructor
    UDPKnowledgeServer(const UDPKnowledgeServer &other) : BaseKnowledgeServer(other) {}

    // Copy assignment
    UDPKnowledgeServer &operator=(const UDPKnowledgeServer &other)
    {
        BaseKnowledgeServer::operator=(other);
        return *this;
    }

    void Start() override
    {
        this->running_ = true;
        this->socket_  = new (this->socket_buffer_.data()) asio::ip::udp::socket(io_context_);
        this->socket_->open(asio::ip::udp::v4());
        this->socket_->bind(asio::ip::udp::endpoint(
            asio::ip::address::from_string(this->robot_model->robot_host.c_str()), this->robot_model->robot_port));

        this->thread_ = std::thread([this]() { this->StartReceive(); });
    }

    void Stop() override
    {
        this->running_ = false;
        this->thread_.join();
        this->socket_->close();
        this->socket_ = nullptr;
    }

private:
    asio::io_context io_context_;
    alignas(asio::ip::udp::socket) std::array<uint8_t, sizeof(asio::ip::udp::socket)> socket_buffer_ = {0};
    asio::ip::udp::socket *socket_                                                                   = nullptr;
    std::thread thread_;
    bool running_               = false;
    static constexpr char TAG[] = "UDPKnowledgeServer";

    void StartReceive()
    {
        ESP_LOGI(TAG, "Starting knowledge connection on %s:%hu", this->robot_model->robot_host.c_str(),
                 this->robot_model->robot_port);
        while (this->running_)
        {
            struct pollfd pfd = {this->socket_->native_handle(), POLLIN, 0};
            int retval        = poll(&pfd, 1, 1000);
            if (retval == 0)
            { // timeout
                ESP_LOGD(TAG, "Timeout, no data received%s", "");
                continue;
            }
            if (retval < 0)
            {
                ESP_LOGE(TAG, "Error receiving data%s", "poll");
                perror("poll");
                continue;
            }

            asio::ip::udp::endpoint client;
            std::array<uint8_t, EpuckKnowledgePacket::calcsize()> data{};
            asio::error_code ec;
            auto received = socket_->receive_from(asio::buffer(data, EpuckKnowledgePacket::calcsize()), client, 0, ec);
            if (received < 1)
            {
                ESP_LOGE(TAG, "No data received%s", "");
                continue;
            }
            HandleReceive(client, data, ec, received);
        }
    }

    void HandleReceive(const asio::ip::udp::endpoint &client,
                       const std::array<uint8_t, EpuckKnowledgePacket::calcsize()> &data, const asio::error_code &error,
                       std::size_t /*bytes_transferred*/)
    {
        if (error)
        {
            ESP_LOGE(TAG, "Error receiving data: %s", error.message().c_str());
            return;
        }

        auto request = EpuckKnowledgePacket::unpack(data.data());

        auto known_ids_before = this->robot_model->GetKnownIds();

        auto num_inserted = this->robot_model->InsertKnownIds(request.known_ids);

        auto known_ids_after = this->robot_model->GetKnownIds();

        if (num_inserted > 0)
        {
            RobotSizeSet<uint8_t> new_ids;
            std::set_difference(request.known_ids.begin(), request.known_ids.end(), known_ids_before.begin(),
                                known_ids_before.end(), std::inserter(new_ids, new_ids.begin()));

            ESP_LOGI(TAG, "Received new IDs from %hhu (%s:%hu): %s", request.robot_id,
                     client.address().to_string().c_str(), client.port(), known_ids_to_string(new_ids).data());
        }

        ESP_LOGD(TAG, "Received knowledge from %hhu (%s:%hu): %s", request.robot_id,
                 client.address().to_string().c_str(), client.port(), known_ids_to_string(request.known_ids).data());

        auto knowledge      = EpuckKnowledgePacket();
        knowledge.robot_id  = this->robot_model->robot_id;
        knowledge.N         = std::distance(known_ids_after.begin(), known_ids_after.end());
        knowledge.known_ids = {known_ids_after.begin(), known_ids_after.end()};

        socket_->send_to(asio::buffer(knowledge.pack(), EpuckKnowledgePacket::calcsize()), client);

        ESP_LOGD(TAG, "Sent knowledge to %hhu (%s:%hu): %s", request.robot_id, client.address().to_string().c_str(),
                 client.port(), known_ids_to_string(knowledge.known_ids).data());
    }
};

class UDPKnowledgeClient : public BaseKnowledgeClient
{
public:
    UDPKnowledgeClient() = default;
    UDPKnowledgeClient(EpuckNeighbourPacket neighbour, std::function<bool()> running,
                       std::shared_ptr<BaseRobotCommsModel> robot_model)
        : BaseKnowledgeClient(neighbour, running, robot_model){};

    // Move constructor
    UDPKnowledgeClient(UDPKnowledgeClient &&other) noexcept : BaseKnowledgeClient(std::move(other)) {}

    // Move assignment
    UDPKnowledgeClient &operator=(UDPKnowledgeClient &&other) noexcept
    {
        BaseKnowledgeClient::operator=(std::move(other));
        return *this;
    }

    // Copy constructor
    UDPKnowledgeClient(const UDPKnowledgeClient &other) : BaseKnowledgeClient(other) {}

    // Copy assignment
    UDPKnowledgeClient &operator=(const UDPKnowledgeClient &other)
    {
        BaseKnowledgeClient::operator=(other);
        return *this;
    }

    void Start() override
    {
        this->stopping_ = false;
        this->client_   = new (this->socket_buffer_.data()) asio::ip::udp::socket(io_context_);
        this->client_->open(asio::ip::udp::v4());
        this->thread_ = std::thread([this]() { this->SendKnowledge(); });
    }

    void Stop() override
    {
        this->stopping_ = true;
        this->thread_.join();
        this->client_->close();
        this->client_ = nullptr;
    }

private:
    asio::io_context io_context_;
    alignas(asio::ip::udp::socket) std::array<uint8_t, sizeof(asio::ip::udp::socket)> socket_buffer_ = {0};
    asio::ip::udp::socket *client_                                                                   = nullptr;
    std::thread thread_;
    bool stopping_              = false;
    static constexpr char TAG[] = "UDPKnowledgeClient";

    void SendKnowledge()
    {
        ESP_LOGI(TAG, "Starting knowledge connection with %hhu (%s:%hu)", this->neighbour.robot_id,
                 this->neighbour.host.c_str(), this->neighbour.port);

        auto server =
            asio::ip::udp::endpoint(asio::ip::address::from_string(this->neighbour.host.c_str()), this->neighbour.port);

        while (this->running() && !this->stopping_)
        {
            auto knowledge      = EpuckKnowledgePacket();
            knowledge.robot_id  = this->robot_model->robot_id;
            knowledge.known_ids = {this->robot_model->GetKnownIds()};
            knowledge.N         = std::distance(knowledge.known_ids.begin(), knowledge.known_ids.end());

            client_->send_to(asio::buffer(knowledge.pack(), EpuckKnowledgePacket::calcsize()), server);

            ESP_LOGD(TAG, "Sent knowledge to %hhu (%s:%hu): %s", this->neighbour.robot_id, this->neighbour.host.c_str(),
                     this->neighbour.port, known_ids_to_string(knowledge.known_ids).data());

            struct pollfd pfd = {this->client_->native_handle(), POLLIN, 0};
            int retval        = poll(&pfd, 1, 20);
            if (retval == 0)
            { // timeout
                ESP_LOGW(TAG, "Timeout waiting for response from %hhu (%s:%hu)", this->neighbour.robot_id,
                         this->neighbour.host.c_str(), this->neighbour.port);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            if (retval < 0)
            {
                ESP_LOGE(TAG, "poll %s", "error");
                perror("poll");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }

            std::array<uint8_t, EpuckKnowledgePacket::calcsize()> data{};
            auto received = this->client_->receive_from(asio::buffer(data), server);
            if (received < 1)
            {
                ESP_LOGW(TAG, "(%s:%hu) disconnected", this->neighbour.host.c_str(), this->neighbour.port);
                break;
            }

            auto response = EpuckKnowledgePacket::unpack(data.data());

            const auto &known_ids_before = knowledge.known_ids;

            auto num_inserted = this->robot_model->InsertKnownIds(response.known_ids);

            if (num_inserted > 0)
            {
                RobotSizeSet<uint8_t> new_ids;
                std::set_difference(response.known_ids.begin(), response.known_ids.end(), known_ids_before.begin(),
                                    known_ids_before.end(), std::inserter(new_ids, new_ids.begin()));

                ESP_LOGI(TAG, "Received new IDs from %hhu (%s:%hu): %s", response.robot_id,
                         this->neighbour.host.c_str(), this->neighbour.port, known_ids_to_string(new_ids).data());
            }

            ESP_LOGD(TAG, "Received knowledge from %hhu (%s:%hu): %s", response.robot_id, this->neighbour.host.c_str(),
                     this->neighbour.port, known_ids_to_string(response.known_ids).data());

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
};
