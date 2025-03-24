#pragma once

#include <asio.hpp>
#include <memory>

// Forward declarations
class ITcpSocket;
class IUdpSocket;
class IResolver;
class IIoContext;
class ITimer;

// Interface for IoContext
class IIoContext
{
public:
    virtual ~IIoContext()                          = default;
    virtual void run()                             = 0;
    virtual void stop()                            = 0;
    virtual asio::io_context &get_native_context() = 0;
};

// Real implementation of IoContext
class IoContext : public IIoContext
{
public:
    IoContext() = default;

    void run() override { _context.run(); }

    void stop() override { _context.stop(); }

    asio::io_context &get_native_context() override { return _context; }

private:
    asio::io_context _context;
};

// Interface for TCP endpoint
class ITCPEndpoint
{
public:
    virtual ~ITCPEndpoint() = default;

    virtual asio::ip::tcp::endpoint &get_native_endpoint() = 0;

    virtual asio::ip::address address() = 0;

    virtual uint16_t port() = 0;
};

// Real implementation of TCP endpoint
class TCPEndpoint : public ITCPEndpoint
{
public:
    TCPEndpoint() = default;

    explicit TCPEndpoint(const asio::ip::address_v4 &addr, uint16_t port)
        : _endpoint(asio::ip::tcp::endpoint(addr, port))
    {
    }

    asio::ip::tcp::endpoint &get_native_endpoint() override { return _endpoint; }

    asio::ip::address address() override { return _endpoint.address(); }

    uint16_t port() override { return _endpoint.port(); }

private:
    asio::ip::tcp::endpoint _endpoint;
};

// Interface for UDP endpoint
class IUDPEndpoint
{
public:
    virtual ~IUDPEndpoint() = default;

    virtual asio::ip::udp::endpoint &get_native_endpoint() = 0;

    virtual asio::ip::address address() = 0;

    virtual uint16_t port() = 0;
};

// Real implementation of UDP endpoint
class UDPEndpoint : public IUDPEndpoint
{
public:
    UDPEndpoint() = default;

    explicit UDPEndpoint(const asio::ip::address_v4 &addr, uint16_t port)
        : _endpoint(asio::ip::udp::endpoint(addr, port))
    {
    }

    asio::ip::udp::endpoint &get_native_endpoint() override { return _endpoint; }

    asio::ip::address address() override { return _endpoint.address(); }

    uint16_t port() override { return _endpoint.port(); }

private:
    asio::ip::udp::endpoint _endpoint;
};

// Interface for TCP socket
class ITcpSocket
{
public:
    virtual ~ITcpSocket() = default;

    virtual void connect(const asio::ip::tcp::endpoint &endpoint) = 0;

    virtual size_t send(const asio::const_buffer &buffer) = 0;

    virtual size_t receive(const asio::mutable_buffer &buffer) = 0;

    virtual void close() = 0;

    virtual bool is_open() = 0;

    virtual asio::ip::tcp::socket &get_native_socket() = 0;
};

// Real implementation of TCP socket
class TcpSocket : public ITcpSocket
{
public:
    explicit TcpSocket(IIoContext &io_context) : _socket(io_context.get_native_context()) {}

    void connect(const asio::ip::tcp::endpoint &endpoint) override { _socket.connect(endpoint); }

    size_t send(const asio::const_buffer &buffer) override { return _socket.send(buffer); }

    size_t receive(const asio::mutable_buffer &buffer) override { return _socket.receive(buffer); }

    void close() override { _socket.close(); }

    bool is_open() override { return _socket.is_open(); }

    asio::ip::tcp::socket &get_native_socket() override { return _socket; }

private:
    asio::ip::tcp::socket _socket;
};

// Interface for UDP socket
class IUdpSocket
{
public:
    virtual ~IUdpSocket() = default;

    virtual void open() = 0;

    virtual void bind(IUDPEndpoint &endpoint) = 0;

    virtual size_t send_to(const asio::const_buffer &buffer, IUDPEndpoint &destination) = 0;

    virtual size_t receive_from(const asio::mutable_buffer &buffer, IUDPEndpoint &sender) = 0;

    virtual void close() = 0;

    virtual bool is_open() = 0;

    virtual asio::ip::udp::socket &get_native_socket() = 0;
};

// Real implementation of UDP socket
class UdpSocket : public IUdpSocket
{
public:
    explicit UdpSocket(IIoContext &io_context) : _socket(io_context.get_native_context()) {}

    void open() override { _socket.open(asio::ip::udp::v4()); }

    void bind(IUDPEndpoint &endpoint) override { _socket.bind(endpoint.get_native_endpoint()); }

    size_t send_to(const asio::const_buffer &buffer, IUDPEndpoint &destination) override
    {
        return _socket.send_to(buffer, destination.get_native_endpoint());
    }

    size_t receive_from(const asio::mutable_buffer &buffer, IUDPEndpoint &sender) override
    {
        return _socket.receive_from(buffer, sender.get_native_endpoint());
    }

    void close() override { _socket.close(); }

    bool is_open() override { return _socket.is_open(); }

    asio::ip::udp::socket &get_native_socket() override { return _socket; }

private:
    asio::ip::udp::socket _socket;
};

// Interface for resolver
class IResolver
{
public:
    virtual ~IResolver() = default;

    virtual asio::ip::tcp::resolver::results_type resolve_tcp(const std::string &host, const std::string &service) = 0;

    virtual asio::ip::udp::resolver::results_type resolve_udp(const std::string &host, const std::string &service) = 0;
};

// Real implementation of resolver
class Resolver : public IResolver
{
public:
    explicit Resolver(IIoContext &io_context)
        : _tcp_resolver(io_context.get_native_context()), _udp_resolver(io_context.get_native_context())
    {
    }

    asio::ip::tcp::resolver::results_type resolve_tcp(const std::string &host, const std::string &service) override
    {
        return _tcp_resolver.resolve(host, service);
    }

    asio::ip::udp::resolver::results_type resolve_udp(const std::string &host, const std::string &service) override
    {
        return _udp_resolver.resolve(host, service);
    }

private:
    asio::ip::tcp::resolver _tcp_resolver;
    asio::ip::udp::resolver _udp_resolver;
};

// Interface for timer
class ITimer
{
public:
    virtual ~ITimer() = default;

    virtual void expires_after(const asio::steady_timer::duration &expiry_time) = 0;
    virtual void expires_at(const asio::steady_timer::time_point &expiry_time)  = 0;

    virtual void wait() = 0;

    virtual void cancel() = 0;
};

// Real implementation of timer
class Timer : public ITimer
{
public:
    explicit Timer(IIoContext &io_context) : _timer(io_context.get_native_context()) {}

    void expires_after(const asio::steady_timer::duration &expiry_time) override { _timer.expires_after(expiry_time); }

    void expires_at(const asio::steady_timer::time_point &expiry_time) override { _timer.expires_at(expiry_time); }

    void wait() override { _timer.wait(); }

    void cancel() override { _timer.cancel(); }

private:
    asio::steady_timer _timer;
};

// The factory interface
class INetworkFactory
{
public:
    virtual ~INetworkFactory() = default;

    virtual std::shared_ptr<IIoContext> create_io_context()                                             = 0;
    virtual std::shared_ptr<ITcpSocket> create_tcp_socket(IIoContext &io_context)                       = 0;
    virtual std::shared_ptr<IUdpSocket> create_udp_socket(IIoContext &io_context)                       = 0;
    virtual std::shared_ptr<IResolver> create_resolver(IIoContext &io_context)                          = 0;
    virtual std::shared_ptr<ITimer> create_timer(IIoContext &io_context)                                = 0;
    virtual std::shared_ptr<ITCPEndpoint> create_tcp_endpoint()                                         = 0;
    virtual std::shared_ptr<ITCPEndpoint> create_tcp_endpoint(asio::ip::address_v4 addr, uint16_t port) = 0;
    virtual std::shared_ptr<IUDPEndpoint> create_udp_endpoint()                                         = 0;
    virtual std::shared_ptr<IUDPEndpoint> create_udp_endpoint(asio::ip::address_v4 addr, uint16_t port) = 0;
};

// Real implementation of the factory
class NetworkFactory : public INetworkFactory
{
public:
    std::shared_ptr<IIoContext> create_io_context() override { return std::make_shared<IoContext>(); }

    std::shared_ptr<ITcpSocket> create_tcp_socket(IIoContext &io_context) override
    {
        return std::make_shared<TcpSocket>(io_context);
    }

    std::shared_ptr<IUdpSocket> create_udp_socket(IIoContext &io_context) override
    {
        return std::make_shared<UdpSocket>(io_context);
    }

    std::shared_ptr<IResolver> create_resolver(IIoContext &io_context) override
    {
        return std::make_shared<Resolver>(io_context);
    }

    std::shared_ptr<ITimer> create_timer(IIoContext &io_context) override
    {
        return std::make_shared<Timer>(io_context);
    }

    std::shared_ptr<ITCPEndpoint> create_tcp_endpoint() override { return std::make_shared<TCPEndpoint>(); }

    std::shared_ptr<ITCPEndpoint> create_tcp_endpoint(asio::ip::address_v4 addr, uint16_t port) override
    {
        return std::make_shared<TCPEndpoint>(addr, port);
    }

    std::shared_ptr<IUDPEndpoint> create_udp_endpoint() override { return std::make_shared<UDPEndpoint>(); }

    std::shared_ptr<IUDPEndpoint> create_udp_endpoint(asio::ip::address_v4 addr, uint16_t port) override
    {
        return std::make_shared<UDPEndpoint>(addr, port);
    }
};