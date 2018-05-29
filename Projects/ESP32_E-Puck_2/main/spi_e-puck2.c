/*

File    : spi_e-puck2.c
Author  : Stefano Morgani
Date    : 10 January 2018
REV 1.0

Functions to configure and use the SPI communication between the main processor (F407) and the ESP32. 
*/
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/spi_slave.h"
#include "button_e-puck2.h"
#include "main_e-puck2.h"
#include "rgb_led_e-puck2.h"
#include "socket_e-puck2.h"

// Hardware VSPI pins.
#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define MAX_BUFF_SIZE 38400 // For the image.
#define SPI_PACKET_MAX_SIZE 4092

const int DATA_SENT_BIT = BIT0;
const int DATA_TX_ERROR_BIT = BIT1;

uint8_t* spi_tx_buff;
uint8_t* spi_rx_buff;
uint8_t* image_buff1;

static EventGroupHandle_t spi_event_group;
static spi_slave_transaction_t transaction;

// Configuration for the SPI bus.
static spi_bus_config_t buscfg = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1
};

// Configuration for the SPI slave interface.
static spi_slave_interface_config_t slvcfg = {
    .mode = 0,							// SPI mode0: CPOL=0, CPHA=0.
    .spics_io_num = PIN_NUM_CS,			// CS pin.
    .queue_size = 2,					// We want to be able to queue 3 transactions at a time.
    .flags = 0,
    //.post_setup_cb=my_post_setup_cb,
    //.post_trans_cb=my_post_trans_cb
};

void spi_set_event_data_sent(void) {
	xEventGroupSetBits(spi_event_group, DATA_SENT_BIT);
}

void spi_set_event_data_tx_error(void) {
	xEventGroupSetBits(spi_event_group, DATA_TX_ERROR_BIT);
}

uint8_t* spi_get_data_ptr(void) {
	return image_buff1;
}

void spi_task(void *pvParameter) {
	esp_err_t ret;
	uint16_t numPackets = 0;
	uint32_t remainingBytes = 0;	
	unsigned int packetId = 0;
	uint8_t rx_err = 0;
	
	memset(spi_rx_buff, 0x01, SPI_PACKET_MAX_SIZE);
	memset(spi_tx_buff, 0x01, SPI_PACKET_MAX_SIZE);

	memset(&transaction, 0, sizeof(transaction));
	transaction.rx_buffer = spi_rx_buff;
	transaction.tx_buffer = spi_tx_buff;
	transaction.length = SPI_PACKET_MAX_SIZE*8; // Always set to maximum bytes expected and then read "trans_len" to know how many bytes the master actually sent.
	transaction.user=(void*)0;	// Optional user parameter for the callback.
	
	for(;;) {
		//vTaskDelay(10 / portTICK_PERIOD_MS);
		//vTaskDelay( (TickType_t)10); /* allows the freeRTOS scheduler to take over if needed */

		spi_tx_buff[0] = button_is_pressed(); // Button status to send to F407.
		spi_tx_buff[1] = 0xB7; // Get image.
		transaction.rx_buffer = spi_rx_buff;
		transaction.trans_len = 0;
		
		ret = spi_slave_transmit(VSPI_HOST, &transaction, portMAX_DELAY);
		assert(ret==ESP_OK);

//		ret = spi_slave_transmit(VSPI_HOST, &transaction, 20/portTICK_RATE_MS);
//		if(ret == ESP_ERR_TIMEOUT) {
//			ret = spi_slave_free(VSPI_HOST);
//			assert(ret==ESP_OK);
//		    ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 1);
//		    assert(ret==ESP_OK);
//			continue;
//		}

//		if(transaction.trans_len > SPI_PACKET_MAX_SIZE*8) {
//			printf("1)%d\r\n", transaction.trans_len);
//		}

		if(transaction.trans_len == 12*8) { // Check the correct number of bytes are received.
//			rgb_set_intensity(LED2, RED_LED, spi_rx_buff[0], 0);
//			rgb_set_intensity(LED2, GREEN_LED, spi_rx_buff[1], 0);
//			rgb_set_intensity(LED2, BLUE_LED, spi_rx_buff[2], 0);
//			rgb_set_intensity(LED4, RED_LED, spi_rx_buff[3], 0);
//			rgb_set_intensity(LED4, GREEN_LED, spi_rx_buff[4], 0);
//			rgb_set_intensity(LED4, BLUE_LED, spi_rx_buff[5], 0);
//			rgb_set_intensity(LED6, RED_LED, spi_rx_buff[6], 0);
//			rgb_set_intensity(LED6, GREEN_LED, spi_rx_buff[7], 0);
//			rgb_set_intensity(LED6, BLUE_LED, spi_rx_buff[8], 0);
//			rgb_set_intensity(LED8, RED_LED, spi_rx_buff[9], 0);
//			rgb_set_intensity(LED8, GREEN_LED, spi_rx_buff[10], 0);
//			rgb_set_intensity(LED8, BLUE_LED, spi_rx_buff[11], 0);
		} else {
			continue;
		}
		
//		rgb_set_intensity(LED2, GREEN_LED, 100, 0);

		numPackets = MAX_BUFF_SIZE/SPI_PACKET_MAX_SIZE;
		remainingBytes = MAX_BUFF_SIZE%SPI_PACKET_MAX_SIZE;
		rx_err = 0;
		for(packetId=0; packetId<numPackets; packetId++) {
			transaction.rx_buffer = &image_buff1[packetId*SPI_PACKET_MAX_SIZE];

			ret = spi_slave_transmit(VSPI_HOST, &transaction, portMAX_DELAY);
			assert(ret==ESP_OK);

//			ret = spi_slave_transmit(VSPI_HOST, &transaction, 20/portTICK_RATE_MS);
//			if(ret == ESP_ERR_TIMEOUT) {
//				ret = spi_slave_free(VSPI_HOST);
//				assert(ret==ESP_OK);
//			    ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 1);
//			    assert(ret==ESP_OK);
//				rx_err = 1;
//				break;
//			}

			if(transaction.trans_len > SPI_PACKET_MAX_SIZE*8) {
				printf("2)%d\r\n", transaction.trans_len);
			}

			if(transaction.trans_len != SPI_PACKET_MAX_SIZE*8) { // Check the correct number of bytes are received.			
				rx_err = 1;
//				rgb_set_intensity(LED4, RED_LED, 100, 0);
				break;
			}
		}
		if(remainingBytes>0 && rx_err==0) {
			transaction.rx_buffer = &image_buff1[packetId*SPI_PACKET_MAX_SIZE];

			ret = spi_slave_transmit(VSPI_HOST, &transaction, portMAX_DELAY);
			assert(ret==ESP_OK);

//			ret = spi_slave_transmit(VSPI_HOST, &transaction, 20/portTICK_RATE_MS);
//			if(ret == ESP_ERR_TIMEOUT) {
//				ret = spi_slave_free(VSPI_HOST);
//				assert(ret==ESP_OK);
//			    ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 1);
//			    assert(ret==ESP_OK);
//				rx_err = 1;
//			}

			if(transaction.trans_len > SPI_PACKET_MAX_SIZE*8) {
				printf("3)%d\r\n", transaction.trans_len);
			}

			if(transaction.trans_len != remainingBytes*8) { // Check the correct number of bytes are received.			
				rx_err = 1;
				//rgb_set_intensity(LED6, RED_LED, 100, 0);
			}			
		}
		
		if(rx_err == 0) {

		//	for(packetId=0; packetId<19200; packetId++) {
		//		if(image_buff1[packetId] == 0) {
//					rgb_set_intensity(LED6, BLUE_LED, 100, 0);
		//		}
		//	}

			socket_set_event_data_ready();
//			rgb_set_intensity(LED8, BLUE_LED, 100, 0);
			xEventGroupWaitBits(spi_event_group, DATA_SENT_BIT|DATA_TX_ERROR_BIT, true, false, portMAX_DELAY);
			//xEventGroupWaitBits(spi_event_group, DATA_SENT_BIT|DATA_TX_ERROR_BIT, true, false, 100/portTICK_RATE_MS);
//			rgb_set_intensity(LED8, BLUE_LED, 0, 0);
		}
		
//		rgb_set_intensity(LED2, GREEN_LED, 0, 0);
//		rgb_set_intensity(LED4, RED_LED, 0, 0);
//		rgb_set_intensity(LED6, RED_LED, 0, 0);
//		rgb_set_intensity(LED6, BLUE_LED, 0, 0);

	}
}

void spi_init(void) {
	esp_err_t ret;

	spi_tx_buff = (uint8_t*) heap_caps_malloc(SPI_PACKET_MAX_SIZE, MALLOC_CAP_DMA);
//	if(spi_tx_buff == NULL) {
//		printf("cannot allocate spi tx buff\r\n");
//		return;
//	} else {
//		printf("spi tx buff allocated\r\n");
//	}
	spi_rx_buff = (uint8_t*) heap_caps_malloc(SPI_PACKET_MAX_SIZE, MALLOC_CAP_DMA);
//	if(spi_rx_buff == NULL) {
//		printf("cannot allocate spi rx buff\r\n");
//		return;
//	} else {
//		printf("spi rx buff allocated\r\n");
//	}
	image_buff1 = (uint8_t*) heap_caps_malloc(MAX_BUFF_SIZE, MALLOC_CAP_DMA);
	if(image_buff1 == NULL) {
		printf("cannot allocate image buff\r\n");
		return;
	} else {
		printf("image buff allocated\r\n");
	}

	spi_event_group = xEventGroupCreate();
	
    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);

    // Initialize the SPI bus.
    ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 1);
    assert(ret==ESP_OK);
}
