#pragma once

#include <algorithm>
#include <asio.hpp>

#include "EpuckPackets.hpp"
#include "NetworkFactory.hpp"
#include "types.hpp"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <sys/time.h>
#include <thread>
#include <type_traits>
#include <utility>

// NOLINTBEGIN(cppcoreguidelines-macro-usage)
#define KNOWN_IDS_STRING_EXAMPLE "id: 65535 (seq: 65535)"
#define KNOWN_IDS_STRING_FMT "id: %hu (seq: %hu)"
// NOLINTEND(cppcoreguidelines-macro-usage)

static inline auto known_ids_set_to_string(const robot_size_set<robot_id_type> &known_ids)
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

template <typename IterRecord>
static inline auto known_ids_to_string(IterRecord begin, IterRecord end)
{
    static std::array<char, ((sizeof(KNOWN_IDS_STRING_EXAMPLE) - 1) * MAX_ROBOTS)
                                + ((sizeof(", ") - 1) * (MAX_ROBOTS - 1)) + sizeof("")>
        output = {0};
    if (begin == end)
    {
        snprintf(output.data(), sizeof("{}"), "{}");
        return output;
    }

    // auto integer_cx  = static_cast<uint8_t>((*begin).centroid.x);
    // auto fraction_cx = static_cast<uint8_t>(1000.0 * ((*begin).centroid.x - integer_cx));
    // auto integer_cy  = static_cast<uint8_t>((*begin).centroid.y);
    // auto fraction_cy = static_cast<uint8_t>(1000.0 * ((*begin).centroid.y - integer_cy));
    // auto integer_cz  = static_cast<uint8_t>((*begin).centroid.z);
    // auto fraction_cz = static_cast<uint8_t>(1000.0 * ((*begin).centroid.z - integer_cz));

    snprintf(output.data(), sizeof(output), KNOWN_IDS_STRING_FMT, (*begin).robot_id, (*begin).seq);
    begin++;
    for (; begin != end; begin++)
    {
        std::array<char, sizeof(", " KNOWN_IDS_STRING_EXAMPLE)> buf = {0};

        // integer_cx  = static_cast<uint8_t>((*begin).centroid.x);
        // fraction_cx = static_cast<uint8_t>(1000.0 * ((*begin).centroid.x - integer_cx));
        // integer_cy  = static_cast<uint8_t>((*begin).centroid.y);
        // fraction_cy = static_cast<uint8_t>(1000.0 * ((*begin).centroid.y - integer_cy));
        // integer_cz  = static_cast<uint8_t>((*begin).centroid.z);
        // fraction_cz = static_cast<uint8_t>(1000.0 * ((*begin).centroid.z - integer_cz));

        snprintf(buf.data(), sizeof(buf), ", " KNOWN_IDS_STRING_FMT, (*begin).robot_id, (*begin).seq);
        strncat(output.data(), buf.data(), sizeof(buf));
    }
    return output;
}

class BaseRobotCommsModel
{
public:
    using known_id_record_type = EpuckKnowledgeRecord;
    using known_ids_type       = robot_size_map<robot_id_type, known_id_record_type>;
    // using known_ids_type_iterator = known_ids_type::iterator;

    template <bool CONST_TYPE>
    class KnownIdsTypeIteratorBase
    {
    private:
        using iter_type = std::conditional_t<CONST_TYPE, known_ids_type::const_iterator, known_ids_type::iterator>;
        iter_type _iter;

    public:
        using difference_type   = std::ptrdiff_t;
        using value_type        = known_id_record_type;
        using pointer           = known_id_record_type *;
        using reference         = known_id_record_type &;
        using iterator_category = std::input_iterator_tag;

        KnownIdsTypeIteratorBase() = default;
        explicit KnownIdsTypeIteratorBase(iter_type iter) : _iter(iter) {}
        KnownIdsTypeIteratorBase(const KnownIdsTypeIteratorBase &other)            = default;
        KnownIdsTypeIteratorBase &operator=(const KnownIdsTypeIteratorBase &other) = default;
        ~KnownIdsTypeIteratorBase()                                                = default;

        bool operator==(const KnownIdsTypeIteratorBase &other) const { return this->_iter == other._iter; }
        bool operator!=(const KnownIdsTypeIteratorBase &other) const { return this->_iter != other._iter; }
        value_type operator*() { return (*_iter).second; }
        reference operator->() { return _iter->second; }
        KnownIdsTypeIteratorBase &operator++()
        {
            ++this->_iter;
            return *this;
        }
        KnownIdsTypeIteratorBase operator++(int)
        {
            auto copy = *this;
            operator++();
            return copy;
        }
    };

    using known_ids_type_iterator       = KnownIdsTypeIteratorBase<false>;
    using known_ids_type_const_iterator = KnownIdsTypeIteratorBase<true>;

    const robot_id_type robot_id;
    const host_size_string manager_host;
    const uint16_t manager_port;
    const host_size_string robot_comms_host;
    const uint16_t robot_comms_request_port;
    const host_size_string robot_knowledge_host;
    const uint16_t robot_knowledge_exchange_port;

    explicit BaseRobotCommsModel(const robot_id_type &robot_id, host_size_string manager_host,
                                 const uint16_t &manager_port, host_size_string robot_comms_host,
                                 const uint16_t &robot_comms_request_port, host_size_string robot_knowledge_host,
                                 const uint16_t &robot_knowledge_exchange_port)
        : robot_id(robot_id), manager_host(std::move(manager_host)), manager_port(manager_port),
          robot_comms_host(std::move(robot_comms_host)), robot_comms_request_port(robot_comms_request_port),
          robot_knowledge_host(std::move(robot_knowledge_host)),
          robot_knowledge_exchange_port(robot_knowledge_exchange_port)
    {
        this->update_record();
    }

    virtual void start()         = 0;
    virtual void stop() noexcept = 0;

    [[nodiscard]] known_ids_type_const_iterator known_ids_begin() const
    {
        return known_ids_type_const_iterator(this->_known_ids.cbegin());
    }

    [[nodiscard]] known_ids_type_const_iterator known_ids_end() const
    {
        return known_ids_type_const_iterator(this->_known_ids.cend());
    }

    [[nodiscard]] size_t known_ids_size() const { return std::distance(known_ids_begin(), known_ids_end()); }

    template <typename RecordContainerIterator>
    size_t insert_known_ids(const RecordContainerIterator begin, const RecordContainerIterator end)
    {
        typename std::iterator_traits<RecordContainerIterator>::iterator_category *_ = nullptr;

        auto size_before = this->known_ids_size();
        // std::copy(begin, end, std::inserter(this->known_ids_, this->known_ids_.end()));
        for (auto record = begin; record != end; record++)
        {
            auto not_found     = this->_known_ids.find(record->robot_id) == this->_known_ids.end();
            auto is_this_robot = record->robot_id == this->robot_id;
            auto is_larger_seq = !not_found && this->_known_ids[record->robot_id].seq < record->seq;

            if (is_this_robot && is_larger_seq)
            {
                set_centroid(record->centroid);
                set_boundary(record->boundary);
                update_record();
                continue;
            }

            if (not_found || is_larger_seq) { this->_known_ids.insert_or_assign(record->robot_id, *record); }
        }
        return this->known_ids_size() - size_before;
    }

    void set_centroid(const Centroid &centroid)
    {
        this->_centroid = centroid;
        update_record();
    }

    void set_boundary(const Boundary &boundary)
    {
        this->_boundary = boundary;
        update_record();
    }

    [[nodiscard]] const Centroid &get_centroid() const { return this->_centroid; }

    [[nodiscard]] const Boundary &get_boundary() const { return this->_boundary; }

    [[nodiscard]] uint16_t get_seq() { return update_record().seq; }

    [[nodiscard]] EpuckKnowledgePacket create_knowledge_packet()
    {
        // Update self record to share the same seq value with this new packet
        const EpuckKnowledgeRecord &record = update_record();

        auto packet          = EpuckKnowledgePacket();
        packet.robot_id      = this->robot_id;
        packet.seq           = record.seq;
        packet.N             = this->_n;
        packet.num_known_ids = this->known_ids_size();
        std::copy(this->known_ids_begin(), this->known_ids_end(), packet.known_ids.begin());

        return packet;
    }

private:
    const EpuckKnowledgeRecord &update_record()
    {
        EpuckKnowledgeRecord new_record = {this->robot_id, this->_centroid, this->_boundary, ++this->_seq};

        // Update the sequence number of the internal record for this robot, so it matches the one
        // in the response
        this->_known_ids.insert_or_assign(new_record.robot_id, new_record);
        return this->_known_ids[this->robot_id];
    }

    known_ids_type _known_ids;
    Centroid _centroid{};
    Boundary _boundary{};

    uint16_t _seq{};
    uint8_t _n = 1;
};

class BaseKnowledgeServer
{
protected:
    std::shared_ptr<BaseRobotCommsModel> _robot_model;
    std::shared_ptr<INetworkFactory> _network_factory;

public:
    BaseKnowledgeServer() = default;
    BaseKnowledgeServer(std::shared_ptr<BaseRobotCommsModel> robot_model,
                        std::shared_ptr<INetworkFactory> network_factory)
        : _robot_model(std::move(robot_model)), _network_factory(std::move(network_factory)){};
    virtual void start()         = 0;
    virtual void stop() noexcept = 0;
};

class BaseKnowledgeClient
{
protected:
    EpuckNeighbourPacket _neighbour{};
    std::function<bool()> _running;
    std::shared_ptr<BaseRobotCommsModel> _robot_model;
    std::shared_ptr<INetworkFactory> _network_factory;

public:
    BaseKnowledgeClient() = default;
    BaseKnowledgeClient(EpuckNeighbourPacket neighbour, std::function<bool()> running,
                        std::shared_ptr<BaseRobotCommsModel> robot_model,
                        std::shared_ptr<INetworkFactory> network_factory)
        : _neighbour(neighbour), _running(std::move(running)), _robot_model(std::move(robot_model)),
          _network_factory(std::move(network_factory)){};
    virtual void start()         = 0;
    virtual void stop() noexcept = 0;
};

template <typename T, typename U>
class RobotCommsModel : public std::enable_shared_from_this<RobotCommsModel<T, U>>, public BaseRobotCommsModel
{
    static_assert(std::is_base_of_v<BaseKnowledgeServer, T>, "T must inherit from BaseKnowledgeServer");
    static_assert(std::is_base_of_v<BaseKnowledgeClient, U>, "U must inherit from BaseKnowledgeClient");

public:
    RobotCommsModel(const robot_id_type &robot_id, host_size_string manager_host, const uint16_t &manager_port,
                    host_size_string robot_comms_host, const uint16_t &robot_comms_request_port,
                    host_size_string robot_knowledge_host, const uint16_t &robot_knowledge_exchange_port,
                    std::shared_ptr<INetworkFactory> network_factory)
        : BaseRobotCommsModel(robot_id, std::move(manager_host), manager_port, std::move(robot_comms_host),
                              robot_comms_request_port, std::move(robot_knowledge_host), robot_knowledge_exchange_port),
          _network_factory(std::move(network_factory))
    {
        this->_io_context = _network_factory->create_io_context();
    }

    ~RobotCommsModel() { this->stop(); }

    void start() override
    {
        this->_knowledge_server = new (this->_knowledge_server_buffer.data()) // NOLINT(cppcoreguidelines-owning-memory)
            T(std::reinterpret_pointer_cast<BaseRobotCommsModel>(this->shared_from_this()), this->_network_factory);
        this->_knowledge_server->start();

        this->_knowledge_clients.clear();

        this->_running = true;

        auto cfg        = esp_pthread_get_default_config();
        cfg.pin_to_core = CORE_1;
        cfg.stack_size  = 8192;
        cfg.thread_name = "robot_comms_exchange_heartbeats";
        ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg));
        this->_comms_heartbeat_thread = std::thread(&RobotCommsModel::launch_exchange_heartbeats, this);

        this->_comms_request_socket = _network_factory->create_udp_socket(*this->_io_context);
        this->_comms_request_socket->open();
        auto address         = asio::ip::make_address_v4(this->robot_comms_host.c_str());
        auto client_endpoint = _network_factory->create_udp_endpoint(address, this->robot_comms_request_port);
        this->_comms_request_socket->bind(*client_endpoint);

        cfg             = esp_pthread_get_default_config();
        cfg.pin_to_core = CORE_1;
        cfg.stack_size  = 8192;
        cfg.thread_name = "robot_comms_request_knowledge";
        ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg));
        this->_comms_request_thread = std::thread(&RobotCommsModel::launch_handle_commands, this);
    }

    void stop() noexcept override
    {
        this->_running = false;
        if (this->_comms_heartbeat_thread.joinable()) { this->_comms_heartbeat_thread.join(); }
        if (this->_comms_request_thread.joinable()) { this->_comms_request_thread.join(); }
        if (this->_knowledge_server)
        {
            this->_knowledge_server->stop();
            this->_knowledge_server->~T();
        }
        for (auto &[_, client] : this->_knowledge_clients)
        {
            client.stop();
        }
        this->_knowledge_clients.clear();
    }

private:
    void launch_exchange_heartbeats()
    {
#if ENABLE_TRY_CATCH
        try
        {
#endif
            this->exchange_heartbeats();
#if ENABLE_TRY_CATCH
        } catch (const std::exception &e)
        {
            ESP_LOGE(TAG, "Error: %s", e.what());
            throw e;
        }
#endif
    }

    /**
     * @brief Exchange heartbeats with the manager, receiving a list of neighbours and connecting to
     * them if necessary.
     *
     * ~2048 byte stack size
     *
     */
    void exchange_heartbeats()
    {
        auto heartbeat_client = this->_network_factory->create_udp_socket(*this->_io_context);
        heartbeat_client->open();

        auto address = asio::ip::make_address_v4(this->manager_host.c_str());

        auto manager_endpoint = this->_network_factory->create_udp_endpoint(address, this->manager_port);

        while (this->_running)
        {
            ESP_LOGD(RobotCommsModel::TAG, "Sending heartbeat to %s:%hu", this->manager_host.c_str(),
                     this->manager_port);

            auto packet     = EpuckHeartbeatPacket();
            packet.robot_id = this->robot_id;
            strncpy(packet.robot_comms_host.data(), this->robot_comms_host.c_str(), MAX_HOST_LEN);
            packet.robot_comms_request_port = this->robot_comms_request_port;
            strncpy(packet.robot_knowledge_host.data(), this->robot_knowledge_host.c_str(), MAX_HOST_LEN);
            packet.robot_knowledge_exchange_port = this->robot_knowledge_exchange_port;

            ESP_LOGD(TAG, "(%s:%hu)", manager_endpoint->address().to_string().c_str(), manager_endpoint->port());
            std::array<uint8_t, sizeof(EpuckHeartbeatPacket)> packed_packet = packet.pack();
            heartbeat_client->send_to(asio::buffer(packed_packet, sizeof(EpuckHeartbeatPacket)), *manager_endpoint);

            struct pollfd pfd = {heartbeat_client->get_native_socket().native_handle(), POLLIN, 0};
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

            ESP_LOGD(TAG, "Receiving heartbeat response%s", "");

            size_t bytes_received = 0;
            size_t expected_bytes = sizeof(EpuckHeartbeatResponsePacket);
            while (bytes_received < expected_bytes)
            {
                bytes_received +=
                    heartbeat_client->receive_from(asio::buffer(response_buffer.data() + bytes_received,
                                                                sizeof(EpuckHeartbeatResponsePacket) - bytes_received),
                                                   *manager_endpoint);

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
                // robot_size_set<robot_id_type> new_ids({neighbour.robot_id});
                // this->InsertKnownIds(new_ids.cbegin(), new_ids.cend());
            }

            // Connect to new robots that are listed in the response if they have a lower ID
            for (const auto *it = response.neighbours.begin();
                 it - response.neighbours.begin() < response.num_neighbours; it++)
            {
                const auto neighbour = *it;
                // Only connect to robots with lower IDs that are not already connected
                if (this->_knowledge_clients.find(neighbour.robot_id) != this->_knowledge_clients.end()
                    || neighbour.robot_id >= this->robot_id)
                {
                    continue;
                }

                ESP_LOGI(TAG, "Starting thread for neighbour " ROBOT_ID_TYPE_FMT " (%s:%hu)", neighbour.robot_id,
                         neighbour.host.data(), neighbour.port);

                U client(
                    neighbour,
                    [this, neighbour]() {
                        return this->_knowledge_clients.find(neighbour.robot_id) != this->_knowledge_clients.end();
                    },
                    std::reinterpret_pointer_cast<BaseRobotCommsModel>(this->shared_from_this()),
                    this->_network_factory);
                this->_knowledge_clients.insert(std::make_pair(neighbour.robot_id, client));
                this->_knowledge_clients[neighbour.robot_id].start();
            }

            // Disconnect from connected robots that are not listed in the response
            std::vector<robot_id_type> disconnected_ids;
            for (const auto &[neighbour_id, _] : this->_knowledge_clients)
            {
                if (std::find_if(response.neighbours.begin(), response.neighbours.end(),
                                 [neighbour_id](auto &neighbour) { return neighbour.robot_id == neighbour_id; })
                    != response.neighbours.end())
                {
                    continue;
                }

                ESP_LOGI(TAG, "Stopping thread for neighbour " ROBOT_ID_TYPE_FMT, neighbour_id);

                this->_knowledge_clients[neighbour_id].stop();
                disconnected_ids.push_back(neighbour_id);
            }

            for (const auto neighbour_id : disconnected_ids)
            {
                this->_knowledge_clients.erase(neighbour_id);
            }

            // Sleep for 1 second
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }

    void launch_handle_commands()
    {
#if ENABLE_TRY_CATCH
        try
        {
#endif
            ESP_LOGI(TAG, "Command server starting on %s:%hu", this->robot_comms_host.c_str(),
                     this->robot_comms_request_port);
            while (this->_running)
            {

                struct timeval tv = {1, 0};
                fd_set readfds;
                FD_ZERO(&readfds);
                FD_SET(this->_comms_request_socket->get_native_socket().native_handle(), &readfds);
                int fds_ready = select(this->_comms_request_socket->get_native_socket().native_handle() + 1, &readfds,
                                       nullptr, nullptr, &tv);
                if (fds_ready == 0)
                { // timeout
                    ESP_LOGD(TAG, "Command server timeout, no data received%s", "");
                    continue;
                }
                if (fds_ready < 0)
                {
                    ESP_LOGE(TAG, "Command server error receiving data - %s", "select");
                    perror("select");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    continue;
                }

                ESP_LOGD(TAG, "Command server received data%s", "");

                auto client = this->_network_factory->create_udp_endpoint();

                ESP_LOGD(TAG, "Command server client created%s", "");

                std::array<uint8_t, sizeof(EpuckCommandPacket)> data{};

                size_t bytes_received = 0;
                size_t expected_bytes = sizeof(EpuckCommandPacket);
                while (bytes_received < expected_bytes)
                {
                    auto received = this->_comms_request_socket->receive_from(
                        asio::buffer(data.data() + bytes_received, sizeof(EpuckCommandPacket) - bytes_received),
                        *client);
                    ESP_LOGD(TAG, "Command server received %zu bytes", received);
                    if (received < 1)
                    {
                        ESP_LOGW(TAG, "Command client (%s:%hu) disconnected", client->address().to_string().c_str(),
                                 client->port());
                        break;
                    }
                    bytes_received += received;
                }

                ESP_LOGD(TAG, "Command server received expected bytes%s", "");

                if (bytes_received != expected_bytes)
                {
                    ESP_LOGW(TAG, "Command server received %zu bytes, expected %zu bytes", bytes_received,
                             expected_bytes);
                    continue;
                }

                this->handle_commands(client, data);
            }
#if ENABLE_TRY_CATCH
        } catch (const std::exception &e)
        {
            ESP_LOGE(TAG, "Error: %s", e.what());
            throw e;
        }
#endif
    }

    void handle_commands(const std::shared_ptr<IUDPEndpoint> &client,
                         const std::array<uint8_t, sizeof(EpuckCommandPacket)> &data)
    {
        auto request = EpuckCommandPacket::unpack(data.data());

        ESP_LOGD(TAG, "Command handler received command from %s:%hu", client->address().to_string().c_str(),
                 client->port());

        switch (request.command)
        {
        case EpuckCommandPacket::EPUCK_COMMAND_REQUEST_KNOWLEDGE: {
            ESP_LOGD(TAG, "Command handler received REQUEST_KNOWLEDGE");

            auto knowledge = this->create_knowledge_packet();

            this->_comms_request_socket->send_to(asio::buffer(knowledge.pack(), sizeof(EpuckKnowledgePacket)), *client);

            ESP_LOGD(TAG, "Command handler sent knowledge to %s:%hu - %s", client->address().to_string().c_str(),
                     client->port(),
                     known_ids_to_string(knowledge.known_ids.cbegin(),
                                         knowledge.known_ids.cbegin() + knowledge.num_known_ids)
                         .data());

            break;
        }
        case EpuckCommandPacket::EPUCK_COMMAND_SET_KNOWLEDGE: {
            ESP_LOGD(TAG, "Command handler received SET_KNOWLEDGE");

            std::array<uint8_t, sizeof(EpuckKnowledgePacket)> data{};

            size_t bytes_received = 0;
            size_t expected_bytes = sizeof(EpuckKnowledgePacket);
            while (bytes_received < expected_bytes)
            {
                auto received = this->_comms_request_socket->receive_from(
                    asio::buffer(data.data() + bytes_received, sizeof(EpuckKnowledgePacket) - bytes_received), *client);
                ESP_LOGD(TAG, "Command handler received %zu bytes", received);
                if (received < 1)
                {
                    ESP_LOGW(TAG, "Command handler client (%s:%hu) disconnected", client->address().to_string().c_str(),
                             client->port());
                    break;
                }
                bytes_received += received;
            }

            if (bytes_received != expected_bytes)
            {
                ESP_LOGW(TAG, "Command handler received %zu bytes, expected %zu bytes", bytes_received, expected_bytes);
                return;
            }

            auto knowledge = EpuckKnowledgePacket::unpack(data.data());
            ESP_LOGD(TAG, "Command handler received knowledge from %s:%hu - %s", client->address().to_string().c_str(),
                     client->port(),
                     known_ids_to_string(knowledge.known_ids.cbegin(),
                                         knowledge.known_ids.cbegin() + knowledge.num_known_ids)
                         .data());

            // Update the knowledge of the robot
            auto this_robot =
                std::find_if(knowledge.known_ids.begin(), knowledge.known_ids.begin() + knowledge.num_known_ids,
                             [this](const auto &record) { return record.robot_id == this->robot_id; });

            if (this_robot == knowledge.known_ids.end())
            {
                ESP_LOGW(TAG, "Command handler missing knowledge for this robot! - " ROBOT_ID_TYPE_FMT, this->robot_id);
                return;
            }

            // Set the seq number for the knowledge of this robot to UINT16_MAX to force an update to the rest of the
            // attributes in the record
            this_robot->seq = UINT16_MAX;
            this->insert_known_ids(knowledge.known_ids.cbegin(),
                                   knowledge.known_ids.cbegin() + knowledge.num_known_ids);

            ESP_LOGI(TAG, "Command handler updated knowledge - %s",
                     known_ids_to_string(this->known_ids_begin(), this->known_ids_end()).data());

            break;
        }
        default: {
            ESP_LOGW(TAG, "Command handler unknown command %u", request.command);
            break;
        }
        }
    }

    alignas(T) std::array<uint8_t, sizeof(T)> _knowledge_server_buffer = {0};
    T *_knowledge_server;
    robot_size_map<robot_id_type, U> _knowledge_clients;

    bool _running = false;

    std::thread _comms_heartbeat_thread;

    std::shared_ptr<INetworkFactory> _network_factory;
    std::shared_ptr<IIoContext> _io_context;
    std::shared_ptr<IUdpSocket> _comms_request_socket = nullptr;
    std::thread _comms_request_thread;

    static constexpr const char *const TAG = "RobotCommsModel";
};
