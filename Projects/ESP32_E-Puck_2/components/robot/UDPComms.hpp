#pragma once

#include "RobotCommsModel.hpp"

class UDPKnowledgeClient;

class UDPKnowledgeServer : public BaseKnowledgeServer
{
public:
    UDPKnowledgeServer() = default;
    template <class T>
    explicit UDPKnowledgeServer(std::shared_ptr<T> robot_model) : BaseKnowledgeServer(robot_model)
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

    void StartReceive()
    {
        asio::ip::udp::endpoint client;
        std::array<uint8_t, EpuckKnowledgePacket::calcsize()> data{};
        socket_->async_receive_from(
            asio::buffer(data), client, [&](const asio::error_code &error, std::size_t bytes_transferred) {
                return this->HandleReceive(std::move(client), std::move(data), error, bytes_transferred);
            });
    }

    void HandleReceive(asio::ip::udp::endpoint &&client, std::array<uint8_t, EpuckKnowledgePacket::calcsize()> &&data,
                       const asio::error_code &error, std::size_t bytes_transferred)
    {
        if (error) { return; }

        auto request = EpuckKnowledgePacket::unpack(data.data());

        auto known_ids       = this->robot_model->GetKnownIds();
        auto other_known_ids = std::set<uint8_t>();
        // auto difference = std::set_difference(request.known_ids.begin(), request.known_ids.end(),
        //                                       ,
        //                                       this->robot_model_->knowledge_clients_.end(), other_known_ids.begin());

        socket_->send_to(asio::buffer("message"), client);
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
    void Start() override {}
    void Stop() override {}
};
