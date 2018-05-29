/*

File    : socket_e-puck2.c
Author  : Stefano Morgani
Date    : 22 March 2018
REV 1.0

Functions to configure and use the socket to exchange data through WiFi.
*/
#include <string.h>
#include <sys/socket.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"

#include "main_e-puck2.h"
#include "spi_e-puck2.h"
#include "esp_log.h"
#include "rgb_led_e-puck2.h"

#define TCP_PORT 1000
#define TAG "socket:"
#define MAX_BUFF_SIZE 38400 // For the image.
#define SPI_PACKET_MAX_SIZE 4092

const int CONNECTED_BIT = BIT0;
const int DISCONNECTED_BIT = BIT1;
const int DATA_READY_BIT = BIT2;

/* FreeRTOS event group to signal when we are connected & ready to send data */
static EventGroupHandle_t socket_event_group;

int get_socket_error_code(int socket)
{
    int result;
    u32_t optlen = sizeof(int);
    int err = getsockopt(socket, SOL_SOCKET, SO_ERROR, &result, &optlen);
    if (err == -1) {
        ESP_LOGE(TAG, "getsockopt failed:%s", strerror(err));
        return -1;
    }
    return result;
}

int show_socket_error_reason(const char *str, int socket)
{
    int err = get_socket_error_code(socket);

    if (err != 0) {
        ESP_LOGW(TAG, "%s socket error %d %s", str, err, strerror(err));
    }

    return err;
}

void socket_task(void *pvParameter) {
	int server_sock, client_sock;;
	struct sockaddr_in server_addr, client_addr;
	socklen_t client_addr_len = sizeof(client_addr);
	uint8_t conn_state = 0;
	uint8_t* data_buff;
	uint16_t num_packets = 0;
	uint32_t remaining_bytes = 0;
	unsigned int packet_id = 0;
    EventBits_t evg_bits;
    uint8_t tx_err = 0;
    uint8_t toggle_led = 0;

    printf("socket_server: waiting for start bit\n");
    xEventGroupWaitBits(socket_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

    server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_sock < 0) {
    	show_socket_error_reason("create_server", server_sock);
    	return;
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(TCP_PORT);
    if (bind(server_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        show_socket_error_reason("bind_server", server_sock);
        close(server_sock);
        return;
    }
    if (listen(server_sock, 1) < 0) {
        show_socket_error_reason("listen_server", server_sock);
        close(server_sock);
        return;
    }

    while(1) {

        evg_bits = xEventGroupGetBits(socket_event_group);
        if (evg_bits & DISCONNECTED_BIT) {
            close(server_sock);
            return;
        }

    	switch(conn_state) {
    		case 0:
    			rgb_set_intensity(LED2, RED_LED, 0, 0);
    			rgb_set_intensity(LED2, GREEN_LED, 100, 0);
    			rgb_set_intensity(LED2, BLUE_LED, 0, 0);
    		    printf("socket_server: waiting for connection\n");
    		    client_sock = accept(server_sock, (struct sockaddr *)&client_addr, &client_addr_len);
    		    if (client_sock < 0) {
    		        show_socket_error_reason("accept_server", client_sock);
    		        close(server_sock);
    		        return;
    		    }
    		    printf("socket_server: connection established\n");
    		    conn_state = 1;

    			rgb_set_intensity(LED2, RED_LED, 0, 0);
    			rgb_set_intensity(LED2, GREEN_LED, 0, 0);
    			rgb_set_intensity(LED2, BLUE_LED, 100, 0);
    			break;

    		case 1:
    		    xEventGroupWaitBits(socket_event_group, DATA_READY_BIT, true, true, portMAX_DELAY);
    		    data_buff = spi_get_data_ptr();
    		    tx_err = 0;

    		    num_packets = MAX_BUFF_SIZE/SPI_PACKET_MAX_SIZE;
    		    remaining_bytes = MAX_BUFF_SIZE%SPI_PACKET_MAX_SIZE;
    			for(packet_id=0; packet_id<num_packets; packet_id++) {
    				if( send(client_sock, &data_buff[SPI_PACKET_MAX_SIZE*packet_id], SPI_PACKET_MAX_SIZE, 0) < 0) {
    					show_socket_error_reason("send_data", client_sock);
    					conn_state = 0;
    					tx_err = 1;
    					break;
    				}
    			}
    			if(remaining_bytes>0 && tx_err==0) {
    				if( send(client_sock, &data_buff[SPI_PACKET_MAX_SIZE*packet_id], remaining_bytes, 0) < 0) {
    					show_socket_error_reason("send_data", client_sock);
    					conn_state = 0;
    					tx_err = 1;
    					//break;
    				}
    			}

    			if(tx_err == 0) {
    				spi_set_event_data_sent();
    				toggle_led = 1 - toggle_led;
    				rgb_set_intensity(LED2, BLUE_LED, toggle_led*100, 0);
    			} else {
    				spi_set_event_data_tx_error();
    			}
    			break;
    	}

    	vTaskDelay( (TickType_t)10); /* allows the freeRTOS scheduler to take over if needed */
    }



}

void socket_set_event_connected(void) {
	xEventGroupSetBits(socket_event_group, CONNECTED_BIT);
}

void socket_set_event_disconnected(void) {
	xEventGroupSetBits(socket_event_group, DISCONNECTED_BIT);
}

void socket_set_event_data_ready(void) {
	xEventGroupSetBits(socket_event_group, DATA_READY_BIT);
}

void socket_init(void) {
	socket_event_group = xEventGroupCreate();
}
