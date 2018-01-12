/*

File    : spi_e-puck2.c
Author  : Stefano Morgani
Date    : 10 January 2018
REV 1.0

Functions to configure and use the SPI communication between the main processor (F407) and the ESP32. 
*/
#include <string.h>

#include "driver/spi_slave.h"
#include "button_e-puck2.h"
#include "main_e-puck2.h"
#include "rgb_led_e-puck2.h"

// Hardware VSPI pins.
#define PIN_NUM_MOSI 23
#define PIN_NUM_MISO 19
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define SPI_PACKET_MAX_SIZE 4092

uint8_t* spi_tx_buff;
uint8_t* spi_rx_buff;

void spi_task(void *pvParameter) {
	esp_err_t ret;

	spi_slave_transaction_t transaction;
	memset(&transaction, 0, sizeof(transaction));
	transaction.rx_buffer = spi_rx_buff;
	transaction.tx_buffer = spi_tx_buff;
	transaction.length = 12*8;
	transaction.user=(void*)0;	// Optional user parameter for the callback.
	
	for(;;) {
		spi_tx_buff[0] = button_is_pressed(); // Button status to send to F407.
		ret = spi_slave_transmit(VSPI_HOST, &transaction, portMAX_DELAY); // Wait until the master start the transaction.
		assert(ret==ESP_OK);
		if(transaction.trans_len == 12*8) { // Check the correct number of bytes are received.
			rgb_set_intensity(LED2, RED_LED, spi_rx_buff[0], 0);
			rgb_set_intensity(LED2, GREEN_LED, spi_rx_buff[1], 0);
			rgb_set_intensity(LED2, BLUE_LED, spi_rx_buff[2], 0);
			rgb_set_intensity(LED4, RED_LED, spi_rx_buff[3], 0);
			rgb_set_intensity(LED4, GREEN_LED, spi_rx_buff[4], 0);
			rgb_set_intensity(LED4, BLUE_LED, spi_rx_buff[5], 0);
			rgb_set_intensity(LED6, RED_LED, spi_rx_buff[6], 0);
			rgb_set_intensity(LED6, GREEN_LED, spi_rx_buff[7], 0);
			rgb_set_intensity(LED6, BLUE_LED, spi_rx_buff[8], 0);
			rgb_set_intensity(LED8, RED_LED, spi_rx_buff[9], 0);
			rgb_set_intensity(LED8, GREEN_LED, spi_rx_buff[10], 0);
			rgb_set_intensity(LED8, BLUE_LED, spi_rx_buff[11], 0);
		}
	}
}

void spi_init(void) {
	esp_err_t ret;

	spi_tx_buff = (uint8_t*) heap_caps_malloc(SPI_PACKET_MAX_SIZE, MALLOC_CAP_DMA);
	spi_rx_buff = (uint8_t*) heap_caps_malloc(SPI_PACKET_MAX_SIZE, MALLOC_CAP_DMA);	

  	// Configuration for the SPI bus.
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    // Configuration for the SPI slave interface.
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,							// SPI mode0: CPOL=0, CPHA=0.
        .spics_io_num = PIN_NUM_CS,			// CS pin.
        .queue_size = 3,					// We want to be able to queue 3 transactions at a time.
        .flags = 0,
        //.post_setup_cb=my_post_setup_cb,
        //.post_trans_cb=my_post_trans_cb
    };

    // Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(PIN_NUM_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);

    // Initialize the SPI bus.
    ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, 1);
    assert(ret==ESP_OK);
}
