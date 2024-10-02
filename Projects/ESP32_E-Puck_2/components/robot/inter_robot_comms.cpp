/*

File    : socket_e-puck2.c
Author  : Stefano Morgani
Date    : 22 March 2018
REV 1.0

Functions to configure and use the socket to exchange data through WiFi.
*/

#define INTER_ROBOT_COMMS_ESP32

#include "RobotCommsModel.hpp"
#include "UDPComms.hpp"
#include <asio.hpp>

#ifdef __cplusplus
extern "C" {
#endif

#include <esp_pthread.h>
#include <memory>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "tcpip_adapter.h"

#include "esp_log.h"
#include "inter_robot_comms.h"
#include "main_e-puck2.h"
#include "rgb_led_e-puck2.h"
#include "socket_e-puck2.h"
#include "spi_e-puck2.h"
#include "uart_e-puck2.h"

#define TCP_PORT 1001
#define TAG "inter_robot_comms:"
#define MAX_BUFF_SIZE 38400 // For the image.
#define SPI_PACKET_MAX_SIZE 4092

static const int CONNECTED_BIT    = BIT0;
static const int DISCONNECTED_BIT = BIT1;

void inter_robot_comms_task(void *pvParameter)
{
    // try
    // {
    uint8_t conn_state = 0;
    EventBits_t evg_bits;

    constexpr uint8_t robot_id        = 0;
    const HostSizeString manager_host = "192.168.0.2";
    constexpr uint16_t manager_port   = 50000;
    // HostSizeString robot_host         = "192.168.0.2";
    constexpr uint16_t robot_port                                                                           = 0;
    std::array<uint8_t, sizeof(RobotCommsModel<UDPKnowledgeServer, UDPKnowledgeClient>)> robot_model_buffer = {0};
    std::shared_ptr<RobotCommsModel<UDPKnowledgeServer, UDPKnowledgeClient>> robot_model                    = nullptr;

    auto check_disconnection = [&]() {
        evg_bits = xEventGroupGetBits(socket_event_group);
        if (evg_bits & DISCONNECTED_BIT)
        {
            if (robot_model)
            {
                robot_model->Stop();
                robot_model.reset();
            }
            conn_state = 0;
        }
    };

    while (1)
    {
        check_disconnection();

        switch (conn_state)
        {
        case 0: // Wait connection to the AP.
        {
            printf("socket_server: waiting for start bit\n");
            xEventGroupWaitBits(socket_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
            conn_state = 1;
            break;
        }

        case 1: // Start the inter-robot comms.
        {
            ESP_LOGI(TAG, "Starting inter-robot comms");
            asio::io_service io_service;
            asio::ip::tcp::resolver resolver(io_service);

            const char *hostname;
            tcpip_adapter_get_hostname(TCPIP_ADAPTER_IF_STA, &hostname);

            // auto robot_host = HostSizeString("");

            // try
            // {
            //     asio::ip::tcp::resolver::query query(hostname, "");
            //     asio::ip::tcp::resolver::iterator it = resolver.resolve(query);
            //     asio::ip::tcp::endpoint endpoint     = *it;
            //     robot_host                           = HostSizeString(endpoint.address().to_string());
            // } catch (const std::exception &e)
            // {
            //     ESP_LOGE(TAG, "Error resolving hostname: %s", e.what());
            //     throw e;
            // }

            tcpip_adapter_ip_info_t ip_info;
            tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);
            const auto robot_host = HostSizeString(ip4addr_ntoa(&ip_info.ip));

            ESP_LOGI(TAG, "Local IP: %s", robot_host.c_str());

            robot_model = std::shared_ptr<RobotCommsModel<UDPKnowledgeServer, UDPKnowledgeClient>>(
                new (robot_model_buffer.data()) RobotCommsModel<UDPKnowledgeServer, UDPKnowledgeClient>(
                    robot_id, manager_host, manager_port, robot_host, robot_port));

            robot_model->Start();
            conn_state = 2;
            break;
        }

        case 2: // Wait for disconnection.
        {
            check_disconnection();
            conn_state = 0;
            break;
        }
        }

        vTaskDelay((TickType_t)10); /* allows the freeRTOS scheduler to take over if needed */
    }
    // } catch (const std::exception &e)
    // {
    //     ESP_LOGE(TAG, "Error: %s", e.what());
    //     throw e;
    // }
}

void inter_robot_comms_init(void)
{
    auto cfg        = esp_pthread_get_default_config();
    cfg.pin_to_core = CORE_1;
    esp_pthread_set_cfg(&cfg);
}

#ifdef __cplusplus
}
#endif
