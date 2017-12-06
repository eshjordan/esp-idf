/*

File    : bluart_e-puck2.c
Author  : Eliot Ferragni
Date    : 6 december 2017
REV 1.0

Functions to init and use a bluetooth-UART translation
*/

#include "driver/uart.h"
#include "soc/uart_struct.h"

#include "main_e-puck2.h"
#include "bluart_e-puck2.h"
#include "rfcomm_e-puck2.h"

void bluart_init(void);

/*
 *	Init the uart used for the bluart task
*/
void bluart_init(void){
	uart_config_t uart_config = {
        .baud_rate 	= BLUART_UART_BAUDRATE,
        .data_bits 	= BLUART_UART_DATABITS,
        .parity 	= BLUART_UART_PARITY,
        .stop_bits 	= BLUART_UART_STOP_BITS,
        .flow_ctrl 	= BLUART_UART_FLOWCTRL,
        .rx_flow_ctrl_thresh = BLUART_UART_FLOWCTRL_THRESHOLD,
    };

    //Configure parameters.
	uart_param_config(BLUART_UART_USED, &uart_config);
    uart_set_pin(BLUART_UART_USED, BLUART_UART_TX_PIN, BLUART_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //circular buffer for tx and rx and no event queue
    uart_driver_install(BLUART_UART_USED, BLUART_UART_BUFFER_SIZE, BLUART_UART_BUFFER_SIZE, 0, NULL, 0);

}

void bluart_task(void *pvParameter){

	bluart_init();

	uint8_t buffer[BLUART_BUFFER_SIZE];
	int32_t len = 0;

    while(1) {
    	vTaskDelay(10 / portTICK_PERIOD_MS);
        //Read data from UART
		len = uart_read_bytes(BLUART_UART_USED, buffer, BLUART_BUFFER_SIZE, 100 / portTICK_RATE_MS);
		//Write to the bluetooth tx buffer
		if(len > 0){
	        while(bluetooth_write(BLUART_BLUETOOTH_CHANNEL_USED, buffer, len) != DATAS_WRITTEN){
	            vTaskDelay(10 / portTICK_PERIOD_MS);
	        }
	    }

	    //read data from bluetooth rx buffer
	    len = bluetooth_read(BLUART_BLUETOOTH_CHANNEL_USED, buffer, BLUART_BUFFER_SIZE);
	    //printf("read from bluetooth %d\n", len);
	    if(len > 0) {
			int32_t sent = uart_write_bytes(BLUART_UART_USED, (const char*) buffer, len);
			//printf("sent over uart = %d\n",sent);
		}
    }
}