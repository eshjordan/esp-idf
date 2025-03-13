#include <boost/asio.hpp>
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
    virtual ~IIoContext()                               = default;
    virtual void run()                                  = 0;
    virtual void stop()                                 = 0;
    virtual boost::asio::io_context &getNativeContext() = 0;
};

// Real implementation of IoContext
class IoContext : public IIoContext
{
public:
    IoContext() : context_() {}

    void run() override { context_.run(); }

    void stop() override { context_.stop(); }

    boost::asio::io_context &getNativeContext() override { return context_; }

private:
    boost::asio::io_context context_;
};

// Interface for TCP socket
class ITcpSocket
{
public:
    virtual ~ITcpSocket() = default;

    virtual void connect(const boost::asio::ip::tcp::endpoint &endpoint)                      = 0;
    virtual void asyncConnect(const boost::asio::ip::tcp::endpoint &endpoint,
                              std::function<void(const boost::system::error_code &)> handler) = 0;

    virtual size_t send(const boost::asio::const_buffer &buffer)                                        = 0;
    virtual void asyncSend(const boost::asio::const_buffer &buffer,
                           std::function<void(const boost::system::error_code &, std::size_t)> handler) = 0;

    virtual size_t receive(const boost::asio::mutable_buffer &buffer)                                      = 0;
    virtual void asyncReceive(const boost::asio::mutable_buffer &buffer,
                              std::function<void(const boost::system::error_code &, std::size_t)> handler) = 0;

    virtual void close()                                    = 0;
    virtual boost::asio::ip::tcp::socket &getNativeSocket() = 0;
};

// Real implementation of TCP socket
class TcpSocket : public ITcpSocket
{
public:
    TcpSocket(IIoContext &ioContext) : socket_(ioContext.getNativeContext()) {}

    void connect(const boost::asio::ip::tcp::endpoint &endpoint) override { socket_.connect(endpoint); }

    void asyncConnect(const boost::asio::ip::tcp::endpoint &endpoint,
                      std::function<void(const boost::system::error_code &)> handler) override
    {
        socket_.async_connect(endpoint, handler);
    }

    size_t send(const boost::asio::const_buffer &buffer) override { return socket_.send(buffer); }

    void asyncSend(const boost::asio::const_buffer &buffer,
                   std::function<void(const boost::system::error_code &, std::size_t)> handler) override
    {
        socket_.async_send(buffer, handler);
    }

    size_t receive(const boost::asio::mutable_buffer &buffer) override { return socket_.receive(buffer); }

    void asyncReceive(const boost::asio::mutable_buffer &buffer,
                      std::function<void(const boost::system::error_code &, std::size_t)> handler) override
    {
        socket_.async_receive(buffer, handler);
    }

    void close() override { socket_.close(); }

    boost::asio::ip::tcp::socket &getNativeSocket() override { return socket_; }

private:
    boost::asio::ip::tcp::socket socket_;
};

// Interface for UDP socket
class IUdpSocket
{
public:
    virtual ~IUdpSocket() = default;

    virtual void open()                                               = 0;
    virtual void bind(const boost::asio::ip::udp::endpoint &endpoint) = 0;

    virtual size_t sendTo(const boost::asio::const_buffer &buffer,
                          const boost::asio::ip::udp::endpoint &destination)                              = 0;
    virtual void asyncSendTo(const boost::asio::const_buffer &buffer, const boost::asio::ip::udp::endpoint &destination,
                             std::function<void(const boost::system::error_code &, std::size_t)> handler) = 0;

    virtual size_t receiveFrom(const boost::asio::mutable_buffer &buffer, boost::asio::ip::udp::endpoint &sender) = 0;
    virtual void asyncReceiveFrom(const boost::asio::mutable_buffer &buffer, boost::asio::ip::udp::endpoint &sender,
                                  std::function<void(const boost::system::error_code &, std::size_t)> handler)    = 0;

    virtual void close()                                    = 0;
    virtual boost::asio::ip::udp::socket &getNativeSocket() = 0;
};

// Real implementation of UDP socket
class UdpSocket : public IUdpSocket
{
public:
    UdpSocket(IIoContext &ioContext) : socket_(ioContext.getNativeContext()) {}

    void open() override { socket_.open(boost::asio::ip::udp::v4()); }

    void bind(const boost::asio::ip::udp::endpoint &endpoint) override { socket_.bind(endpoint); }

    size_t sendTo(const boost::asio::const_buffer &buffer, const boost::asio::ip::udp::endpoint &destination) override
    {
        return socket_.send_to(buffer, destination);
    }

    void asyncSendTo(const boost::asio::const_buffer &buffer, const boost::asio::ip::udp::endpoint &destination,
                     std::function<void(const boost::system::error_code &, std::size_t)> handler) override
    {
        socket_.async_send_to(buffer, destination, handler);
    }

    size_t receiveFrom(const boost::asio::mutable_buffer &buffer, boost::asio::ip::udp::endpoint &sender) override
    {
        return socket_.receive_from(buffer, sender);
    }

    void asyncReceiveFrom(const boost::asio::mutable_buffer &buffer, boost::asio::ip::udp::endpoint &sender,
                          std::function<void(const boost::system::error_code &, std::size_t)> handler) override
    {
        socket_.async_receive_from(buffer, sender, handler);
    }

    void close() override { socket_.close(); }

    boost::asio::ip::udp::socket &getNativeSocket() override { return socket_; }

private:
    boost::asio::ip::udp::socket socket_;
};

// Interface for resolver
class IResolver
{
public:
    virtual ~IResolver() = default;

    virtual boost::asio::ip::tcp::resolver::results_type resolveTcp(const std::string &host,
                                                                    const std::string &service) = 0;

    virtual void asyncResolveTcp(
        const std::string &host, const std::string &service,
        std::function<void(const boost::system::error_code &, const boost::asio::ip::tcp::resolver::results_type &)>
            handler) = 0;

    virtual boost::asio::ip::udp::resolver::results_type resolveUdp(const std::string &host,
                                                                    const std::string &service) = 0;

    virtual void asyncResolveUdp(
        const std::string &host, const std::string &service,
        std::function<void(const boost::system::error_code &, const boost::asio::ip::udp::resolver::results_type &)>
            handler) = 0;
};

// Real implementation of resolver
class Resolver : public IResolver
{
public:
    Resolver(IIoContext &ioContext)
        : tcpResolver_(ioContext.getNativeContext()), udpResolver_(ioContext.getNativeContext())
    {
    }

    boost::asio::ip::tcp::resolver::results_type resolveTcp(const std::string &host,
                                                            const std::string &service) override
    {
        return tcpResolver_.resolve(host, service);
    }

    void asyncResolveTcp(
        const std::string &host, const std::string &service,
        std::function<void(const boost::system::error_code &, const boost::asio::ip::tcp::resolver::results_type &)>
            handler) override
    {
        tcpResolver_.async_resolve(host, service, handler);
    }

    boost::asio::ip::udp::resolver::results_type resolveUdp(const std::string &host,
                                                            const std::string &service) override
    {
        return udpResolver_.resolve(host, service);
    }

    void asyncResolveUdp(
        const std::string &host, const std::string &service,
        std::function<void(const boost::system::error_code &, const boost::asio::ip::udp::resolver::results_type &)>
            handler) override
    {
        udpResolver_.async_resolve(host, service, handler);
    }

private:
    boost::asio::ip::tcp::resolver tcpResolver_;
    boost::asio::ip::udp::resolver udpResolver_;
};

// Interface for timer
class ITimer
{
public:
    virtual ~ITimer() = default;

    virtual void expires_after(const boost::asio::steady_timer::duration &expiry_time) = 0;
    virtual void expires_at(const boost::asio::steady_timer::time_point &expiry_time)  = 0;

    virtual void wait()                                                                    = 0;
    virtual void asyncWait(std::function<void(const boost::system::error_code &)> handler) = 0;

    virtual void cancel() = 0;
};

// Real implementation of timer
class Timer : public ITimer
{
public:
    Timer(IIoContext &ioContext) : timer_(ioContext.getNativeContext()) {}

    void expires_after(const boost::asio::steady_timer::duration &expiry_time) override
    {
        timer_.expires_after(expiry_time);
    }

    void expires_at(const boost::asio::steady_timer::time_point &expiry_time) override
    {
        timer_.expires_at(expiry_time);
    }

    void wait() override { timer_.wait(); }

    void asyncWait(std::function<void(const boost::system::error_code &)> handler) override
    {
        timer_.async_wait(handler);
    }

    void cancel() override { timer_.cancel(); }

private:
    boost::asio::steady_timer timer_;
};

// The factory interface
class INetworkFactory
{
public:
    virtual ~INetworkFactory() = default;

    virtual std::unique_ptr<IIoContext> createIoContext()                      = 0;
    virtual std::unique_ptr<ITcpSocket> createTcpSocket(IIoContext &ioContext) = 0;
    virtual std::unique_ptr<IUdpSocket> createUdpSocket(IIoContext &ioContext) = 0;
    virtual std::unique_ptr<IResolver> createResolver(IIoContext &ioContext)   = 0;
    virtual std::unique_ptr<ITimer> createTimer(IIoContext &ioContext)         = 0;
};

// Real implementation of the factory
class NetworkFactory : public INetworkFactory
{
public:
    std::unique_ptr<IIoContext> createIoContext() override { return std::make_unique<IoContext>(); }

    std::unique_ptr<ITcpSocket> createTcpSocket(IIoContext &ioContext) override
    {
        return std::make_unique<TcpSocket>(ioContext);
    }

    std::unique_ptr<IUdpSocket> createUdpSocket(IIoContext &ioContext) override
    {
        return std::make_unique<UdpSocket>(ioContext);
    }

    std::unique_ptr<IResolver> createResolver(IIoContext &ioContext) override
    {
        return std::make_unique<Resolver>(ioContext);
    }

    std::unique_ptr<ITimer> createTimer(IIoContext &ioContext) override { return std::make_unique<Timer>(ioContext); }
};