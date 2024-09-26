#pragma once

#include "types.hpp"
#ifdef __cplusplus
extern "C++" {
#endif

#include "RobotCommsModel.hpp"

class UDPKnowledgeServer : public BaseKnowledgeServer
{
public:
    template <typename T, typename U> explicit UDPKnowledgeServer(std::shared_ptr<RobotCommsModel<T, U>> robot_model)
        : BaseKnowledgeServer(robot_model)
    {
        asio::io_context io_context;
        this->socket = asio::ip::udp::socket(io_context, asio::ip::udp::endpoint(asio::ip::udp::v4(), 13));
    };

    void start() {}

    void stop() {}

private:
    asio::ip::udp::socket socket;

    void start_receive()
    {
        asio::ip::udp::endpoint client;
        std::array<uint8_t, EpuckKnowledgePacket::calcsize()> data;
        socket.async_receive_from(
            asio::buffer(data), client, [&](const asio::error_code &error, std::size_t bytes_transferred) {
                return this->handle_receive(std::move(client), std::move(data), error, bytes_transferred);
            });
    }

    void handle_receive(asio::ip::udp::endpoint &&client, std::array<uint8_t, EpuckKnowledgePacket::calcsize()> &&data,
                        const asio::error_code &error, std::size_t bytes_transferred)
    {
        if (error) { return; }

        socket.send_to(asio::buffer("message"), client);
        // socket.async_send_to(asio::buffer("message"), remote_endpoint,
        //                      [this](const asio::error_code &error, std::size_t bytes_transferred) {
        //                          return this->handle_send(error, bytes);
        //                      });

        start_receive();
    }
};

class UDPKnowledgeClient : public BaseKnowledgeClient
{
public:
    template <typename T, typename U> UDPKnowledgeClient(EpuckNeighbourPacket neighbour, bool (*running)(),
                                                         std::shared_ptr<RobotCommsModel<T, U>> robot_model)
        : BaseKnowledgeClient(neighbour, running, robot_model){};
    void start() {}
    void stop() {}
};

#ifdef __cplusplus
}
#endif
