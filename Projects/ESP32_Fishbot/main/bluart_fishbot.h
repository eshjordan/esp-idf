/*

File    : bluart_fishbot.h
Author  : Eliot Ferragni, Daniel Burnier
Date    : 6 december 2017, 4 february 2022
REV 1.0

Functions to init and use a bluetooth-UART translation
*/

#ifndef BLUART_FISHBOT_H
#define BLUART_FISHBOT_H

#include "driver/uart.h"
#include "rfcomm_fishbot.h"

#define BLUART_BUFFER_SIZE 						2000
#define BLUART_UART_BUFFER_SIZE					2000 

#define BLUART_SAMD_UART_BAUDRATE				230400
#define BLUART_SAMD_UART_DATABITS				UART_DATA_8_BITS
#define BLUART_SAMD_UART_PARITY					UART_PARITY_DISABLE
#define BLUART_SAMD_UART_STOP_BITS				UART_STOP_BITS_1
#define BLUART_SAMD_UART_FLOWCTRL				UART_HW_FLOWCTRL_DISABLE
#define BLUART_SAMD_UART_FLOWCTRL_THRESHOLD		122

typedef enum{
	BLUART_SAMD_BLUETOOTH = 0,
	BLUART_BLUETOOTH_SAMD,
	NB_BLUART,
} BLUART_NB;

//struct to represent a bluetooth-uart bidirectional communication
typedef struct {

	CHANNEL_NB bluetooth_channel_rx;		/*!< bluetooth channel used for rx*/
	CHANNEL_NB bluetooth_channel_tx;		/*!< bluetooth channel used for tx*/
	uart_port_t uart_port;				/*!< uart port used*/
	int uart_tx_pin;					/*!< uart tx pin used*/	
	int uart_rx_pin;					/*!< uart rx pin used*/

	uart_config_t* uart_config;			/*!< uart config*/
	gpio_config_t* gpio_status_config;	/*!< gpio config*/
	gpio_num_t gpio_status_pin;			/*!< gpio status pin*/

	esp_err_t (*gpio_set_level_func)(gpio_num_t gpio_num, uint32_t level);	/*!< gpio_set_level pointer*/
	void (*uart_to_bluetooth_func)(void *pvParameter);		/*!< Pointer to a task to run*/
	void (*bluetooth_to_uart_func)(void *pvParameter);		/*!< Pointer to a task to run*/

} bluart_config_t;

/*
 *	Init the uart used for the bluart tasks and create the tasks
*/
void bluart_init(void);


#endif /* BLUART_FISHBOT_H */