/*

File    : bluart_fishbot.c
Author  : Eliot Ferragni, Daniel Burnier
Date    : 6 december 2017, 4 february 2022
REV 1.0

Functions to init and use a bluetooth-UART translation
*/

#include "driver/uart.h"
#include "soc/uart_struct.h"

#include "main_fishbot.h"
#include "bluart_fishbot.h"
#include "rfcomm_fishbot.h"

#define BLUART_UART_TO_BLUETOOTH_TASK_SIZE		5240
#define BLUART_UART_TO_BLUETOOTH_TASK_PRIO		5

#define BLUART_BLUETOOTH_TO_UART_TASK_SIZE		5240
#define BLUART_BLUETOOTH_TO_UART_TASK_PRIO		5

#define BLUART_CONNECTED						0
#define BLUART_NOT_CONNECTED					1

#include "driver/gpio.h"

void bluart_SAMD_uart_to_bluetooth_task(void *pvParameter);
void bluart_SAMD_bluetooth_to_uart_task(void *pvParameter);

/*
 * Function with the same template as the gpio_set_level function but which does nothing.
 * Usefull because one instance of bluart has a pin to tell its connection status and the other not.
 * By using a function pointer in the structure of the bluart instances, we can choose to call the real function
 * or this one, and thus, to modify or not a gpio level.
*/
static esp_err_t void_set_level(gpio_num_t gpio_num, uint32_t level){
	(void)gpio_num;
	(void)level;
	return ESP_OK;
}

/*
 * Same as void_set_level() but in the case where many bluart share the same status pin, Then we indicate a 
 * connected status if one of them is connected.
 * Use the pin to use in the structure of the SAMD bluart
 */
static esp_err_t shared_set_level(gpio_num_t gpio_num, uint32_t level){
	(void)gpio_num;
	(void)level;
	static int8_t status_SAMD = BLUART_NOT_CONNECTED;

	if(gpio_num == BLUART_SAMD_CONNECTION_STATUS_PIN){
		status_SAMD = level;
	}

	if(status_SAMD == BLUART_NOT_CONNECTED){
		gpio_set_level(BLUART_SAMD_CONNECTION_STATUS_PIN, BLUART_NOT_CONNECTED);
	}else{
		gpio_set_level(BLUART_SAMD_CONNECTION_STATUS_PIN, BLUART_CONNECTED);
	}
	return ESP_OK;
}

//uart config of the bluetooth-uart translator instance with the SAMD
uart_config_t uart_SAMD_config = {
	.baud_rate 	= BLUART_SAMD_UART_BAUDRATE,
	.data_bits 	= BLUART_SAMD_UART_DATABITS,
	.parity 	= BLUART_SAMD_UART_PARITY,
	.stop_bits 	= BLUART_SAMD_UART_STOP_BITS,
	.flow_ctrl 	= BLUART_SAMD_UART_FLOWCTRL,
	.rx_flow_ctrl_thresh = BLUART_SAMD_UART_FLOWCTRL_THRESHOLD,
};

//gpio config of the pin used to tell to the SAMD if the bluetooth channel is connected or not
gpio_config_t gpio_SAMD_config = {
	//interrupt of falling edge
	.intr_type = GPIO_PIN_INTR_DISABLE,
	//bit mask of the pins
	.pin_bit_mask = ((uint64_t)1 << BLUART_SAMD_CONNECTION_STATUS_PIN),
	//set as input mode    
	.mode = GPIO_MODE_OUTPUT_OD,
	//enable pull-up mode (no pull-up on pin 34 to 39)
	.pull_up_en = GPIO_PULLUP_ENABLE,
	//disable pull-down mode (no pull-down on pin 34 to 39)
	.pull_down_en = GPIO_PULLDOWN_DISABLE,
};

//config of the uart-bluetooth instances
bluart_config_t bluart_channel[NB_BLUART] = {
	//BLUART_SAMD
	{
		.bluetooth_channel 		= BLUART_SAMD_BLUETOOTH_CHANNEL_USED,		
		.uart_port 				= BLUART_SAMD_UART_USED,				
		.uart_tx_pin 			= BLUART_SAMD_UART_TX_PIN,						
		.uart_rx_pin 			= BLUART_SAMD_UART_RX_PIN,								

		.uart_config 			= &uart_SAMD_config,			
		.gpio_status_config		= &gpio_SAMD_config,	
		.gpio_status_pin		= BLUART_SAMD_CONNECTION_STATUS_PIN,

		.gpio_set_level_func 	= &shared_set_level,
		.uart_to_bluetooth_func = &bluart_SAMD_uart_to_bluetooth_task,
		.bluetooth_to_uart_func = &bluart_SAMD_bluetooth_to_uart_task,
	},
};


void bluart_init(void){
	for(int i = 0 ; i < NB_BLUART ; i++){
		bluart_config_t* bluart = &bluart_channel[i];

		//Configure parameters.
		uart_param_config(bluart->uart_port, bluart->uart_config);
	    uart_set_pin(bluart->uart_port, 
	    	bluart->uart_tx_pin, bluart->uart_rx_pin, 
	    	UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	    //circular buffer for tx and rx and no event queue
	    uart_driver_install(bluart->uart_port, BLUART_UART_BUFFER_SIZE, BLUART_UART_BUFFER_SIZE, 0, NULL, 0);

	    //configure GPIO0 pin in order to use it to inform whether the bluetooth channel is connected or not.
	    //usefull for GDB to determine if a deconnection event need to be done
	    //pin as output opendrain with pull-up
	    
	    if(bluart->gpio_status_config != NULL){
	    	gpio_config(bluart->gpio_status_config);
			gpio_set_level(bluart->gpio_status_pin, BLUART_NOT_CONNECTED);
	    }
	    
	    //creates the tasks to handle the UART-Bluetooth pipelines
	    xTaskCreatePinnedToCore(bluart->uart_to_bluetooth_func, "uart to bluetooth translator", 
	              BLUART_UART_TO_BLUETOOTH_TASK_SIZE, NULL, BLUART_UART_TO_BLUETOOTH_TASK_PRIO, NULL, CORE_1);
		
		xTaskCreatePinnedToCore(bluart->bluetooth_to_uart_func, "bluetooth to uart translator", 
	              BLUART_BLUETOOTH_TO_UART_TASK_SIZE, NULL, BLUART_BLUETOOTH_TO_UART_TASK_PRIO, NULL, CORE_1);

	}
}

void bluart_generic_uart_to_bluetooth_task(bluart_config_t* bluart, uint8_t* buffer, int32_t* len, int16_t* status){
	vTaskDelay(1 / portTICK_PERIOD_MS);
    //Read data from UART
	*len = uart_read_bytes(bluart->uart_port, buffer, BLUART_BUFFER_SIZE, DELAY_1_TICKS);
	//Write to the bluetooth tx buffer
	if(*len > 0){
		bool not_written = true;
        while(not_written) {
        	gpio_set_level(GPIO_A6, 0); // -> Write
        	gpio_set_level(GPIO_A7, 1); // -> Start
        	not_written = (bluetooth_write(bluart->bluetooth_channel, buffer, *len, status) != DATAS_WRITTEN);
        	gpio_set_level(GPIO_A7, 0); // -> Stop
        	//if bluetooth is not connected, we skip the sending
        	//it is like flushing the datas if nobody is listening
        	if(*status == BLUETOOTH_NOT_CONNECTED){
		    	 break;
		    	}
            //vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
}

void bluart_generic_bluetooth_to_uart_task(bluart_config_t* bluart, uint8_t* buffer, int32_t* len, int16_t* status){
	vTaskDelay(1 / portTICK_PERIOD_MS);
    //read data from bluetooth rx buffer
    *len = bluetooth_read(bluart->bluetooth_channel, buffer, BLUART_BUFFER_SIZE, status);
    //updates the pin to tell the bluetooth is connected
    if(*status == BLUETOOTH_NOT_CONNECTED){
    	bluart->gpio_set_level_func(bluart->gpio_status_pin, BLUART_NOT_CONNECTED);
    }else{
    	bluart->gpio_set_level_func(bluart->gpio_status_pin, BLUART_CONNECTED);
    }
	//write to UART
    if(*len > 0) {
	  	gpio_set_level(GPIO_A6, 1); // -> Read
	  	gpio_set_level(GPIO_A7, 1); // -> Start
			uart_write_bytes(bluart->uart_port, (const char*) buffer, *len);
	  	gpio_set_level(GPIO_A7, 0); // -> Stop
	}
}
/*
 *	Tasks to read from UART and write to Bluetooth
*/
void bluart_SAMD_uart_to_bluetooth_task(void *pvParameter){
	uint8_t buffer[BLUART_BUFFER_SIZE];
	int32_t len = 0;
	int16_t status;

    while(1) {
    	bluart_generic_uart_to_bluetooth_task(&bluart_channel[BLUART_SAMD], buffer, &len, &status);
    }
}

/*
 *	Tasks to read from Bluetooth and write to UART
 *	Can tell if the bluetooth is connected or not
*/
void bluart_SAMD_bluetooth_to_uart_task(void *pvParameter){
	uint8_t buffer[BLUART_BUFFER_SIZE];
	int32_t len = 0;
	int16_t status;

    while(1) {
    	bluart_generic_bluetooth_to_uart_task(&bluart_channel[BLUART_SAMD], buffer, &len, &status);
	}
}