#pragma once

#include "EpuckPackets.hpp"
#include "RobotCommsModel.hpp"
#include "types.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <sstream>

template <class T> using RobotIdListAllocator =
    static_allocator<T, ((sizeof("255") - 1) * MAX_ROBOTS) + ((sizeof(", ") - 1) * (MAX_ROBOTS - 1)) + sizeof("")>;

static auto known_ids_to_string(const RobotSizeSet<uint8_t> &known_ids)
{
    using RobotIdListStringStream = std::basic_stringstream<char, std::char_traits<char>, RobotIdListAllocator<char>>;
    RobotIdListStringStream oss;
    oss << *known_ids.begin();
    for (auto it = known_ids.begin(); ++it != known_ids.end();)
    {
        oss << ", " << *it;
    }
    return oss.str();
}

class UDPKnowledgeClient;

class UDPKnowledgeServer : public BaseKnowledgeServer
{
public:
    UDPKnowledgeServer() = default;
    template <class T> explicit UDPKnowledgeServer(std::shared_ptr<T> robot_model) : BaseKnowledgeServer(robot_model)
    {
        asio::io_context io_context;
        this->socket_ =
            std::shared_ptr<asio::ip::udp::socket>(new (this->socket_buffer_) asio::ip::udp::socket(io_context));
    }

    void Start() override
    {
        this->socket_->open(asio::ip::udp::v4());
        this->socket_->bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), 1001));
        StartReceive();
    }

    void Stop() override { this->socket_->close(); }

private:
    alignas(asio::ip::udp::socket) uint8_t socket_buffer_[sizeof(asio::ip::udp::socket)] = {0};
    std::shared_ptr<asio::ip::udp::socket> socket_;
    static constexpr char TAG[] = "UDPKnowledgeServer";

    void StartReceive()
    {
        asio::ip::udp::endpoint client;
        std::array<uint8_t, EpuckKnowledgePacket::calcsize()> data{};
        socket_->async_receive_from(asio::buffer(data), client,
                                    [&](const asio::error_code &error, std::size_t bytes_transferred) {
                                        return this->HandleReceive(std::move(client), data, error, bytes_transferred);
                                    });
    }

    void HandleReceive(asio::ip::udp::endpoint &&client,
                       const std::array<uint8_t, EpuckKnowledgePacket::calcsize()> &data, const asio::error_code &error,
                       std::size_t bytes_transferred)
    {
        if (error)
        {
            ESP_LOGE(TAG, "Error receiving data: %s", error.message().c_str());
            return;
        }

        auto request = EpuckKnowledgePacket::unpack(data.data());

        auto known_ids_before = this->robot_model->GetKnownIds();

        auto num_inserted = this->robot_model->InsertKnownIds(request.known_ids.begin(), request.known_ids.end());

        auto known_ids_after = this->robot_model->GetKnownIds();

        if (num_inserted > 0)
        {
            RobotSizeSet<uint8_t> new_ids;
            std::set_difference(request.known_ids.begin(), request.known_ids.end(), known_ids_before.first,
                                known_ids_before.second, std::inserter(new_ids, new_ids.begin()));

            ESP_LOGI(TAG, "Received new IDs from %hhu (%s:%hu): %s", request.robot_id,
                     client.address().to_string().c_str(), client.port(), known_ids_to_string(new_ids).c_str());
        }

        ESP_LOGD(TAG, "Received knowledge from %hhu (%s:%hu): %s", request.robot_id,
                 client.address().to_string().c_str(), client.port(), known_ids_to_string(request.known_ids).c_str());

        auto knowledge      = EpuckKnowledgePacket();
        knowledge.robot_id  = this->robot_model->robot_id;
        knowledge.N         = std::distance(known_ids_after.first, known_ids_after.second);
        knowledge.known_ids = {known_ids_after.first, known_ids_after.second};

        socket_->send_to(asio::buffer(knowledge.pack(), EpuckKnowledgePacket::calcsize()), client);

        ESP_LOGD(TAG, "Sent knowledge to %hhu (%s:%hu): %s", request.robot_id, client.address().to_string().c_str(),
                 client.port(), known_ids_to_string(knowledge.known_ids).c_str());

        // socket.async_send_to(asio::buffer("message"), remote_endpoint,
        //                      [this](const asio::error_code &error, std::size_t bytes_transferred) {
        //                          return this->handle_send(error, bytes);
        //                      });

        StartReceive();
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
        asio::io_context io_context;
        this->client_ =
            std::shared_ptr<asio::ip::udp::socket>(new (this->socket_buffer_) asio::ip::udp::socket(io_context));
        this->client_->open(asio::ip::udp::v4());
        this->thread_ = std::thread([this]() { this->SendKnowledge(); });
    }

    void Stop() override
    {
        this->client_->close();
        this->client_.reset();
        this->thread_.join();
    }

private:
    alignas(asio::ip::udp::socket) uint8_t socket_buffer_[sizeof(asio::ip::udp::socket)] = {0};
    std::shared_ptr<asio::ip::udp::socket> client_;
    std::thread thread_;
    static constexpr char TAG[] = "UDPKnowledgeClient";

    void SendKnowledge()
    {
        ESP_LOGI(TAG, "Starting knowledge connection with %hhu (%s:%hu)", this->neighbour.robot_id,
                 this->neighbour.host.c_str(), this->neighbour.port);

        auto server =
            asio::ip::udp::endpoint(asio::ip::address::from_string(this->neighbour.host.c_str()), this->neighbour.port);

        while (this->running())
        {
            auto knowledge     = EpuckKnowledgePacket();
            knowledge.robot_id = this->robot_model->robot_id;
            knowledge.N =
                std::distance(this->robot_model->GetKnownIds().first, this->robot_model->GetKnownIds().second);
            knowledge.known_ids = {this->robot_model->GetKnownIds().first, this->robot_model->GetKnownIds().second};

            client_->send_to(asio::buffer(knowledge.pack(), EpuckKnowledgePacket::calcsize()), server);

            ESP_LOGD(TAG, "Sent knowledge to %hhu (%s:%hu): %s", this->neighbour.robot_id, this->neighbour.host.c_str(),
                     this->neighbour.port, known_ids_to_string(knowledge.known_ids).c_str());

            std::array<uint8_t, EpuckKnowledgePacket::calcsize()> data{};
            auto received = this->client_->receive_from(asio::buffer(data), server);
            if (received < 1)
            {
                ESP_LOGW(TAG, "(%s:%hu) disconnected", this->neighbour.host.c_str(), this->neighbour.port);
                break;
            }

            auto response = EpuckKnowledgePacket::unpack(data.data());

            auto known_ids_before = this->robot_model->GetKnownIds();

            auto num_inserted = this->robot_model->InsertKnownIds(response.known_ids.begin(), response.known_ids.end());

            auto known_ids_after = this->robot_model->GetKnownIds();

            if (num_inserted > 0)
            {
                RobotSizeSet<uint8_t> new_ids;
                std::set_difference(response.known_ids.begin(), response.known_ids.end(), known_ids_before.first,
                                    known_ids_before.second, std::inserter(new_ids, new_ids.begin()));

                ESP_LOGI(TAG, "Received new IDs from %hhu (%s:%hu): %s", response.robot_id,
                         this->neighbour.host.c_str(), this->neighbour.port, known_ids_to_string(new_ids).c_str());
            }

            ESP_LOGD(TAG, "Received knowledge from %hhu (%s:%hu): %s", response.robot_id, this->neighbour.host.c_str(),
                     this->neighbour.port, known_ids_to_string(response.known_ids).c_str());
        }
    }
};
