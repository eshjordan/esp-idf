/*

File    : main_e-puck2.h
Author  : Eliot Ferragni
Date    : 12 november 2017
REV 1.0

Firmware to be run on the ESP32 of the e-puck2
*/


#ifndef MAIN_E_PUCK_2_H
#define MAIN_E_PUCK_2_H

#define DELAY_1_TICKS              	1
#define DELAY_10_TICKS              10
#define DELAY_1000_TICKS            1000   

#define CORE_0						0
#define CORE_1						1


///////////////////////////////////////////BLUART DEFINITIONS//////////////////////////////////////////////
#define BLUART_746_BLUETOOTH_CHANNEL_USED		CHANNEL_1

#define BLUART_746_UART_USED					UART_NUM_0
#define BLUART_746_UART_TX_PIN					UART_PIN_NO_CHANGE //default pin U0TXD
#define BLUART_746_UART_RX_PIN					UART_PIN_NO_CHANGE //default pin U0RXD

#define BLUART_746_CONNECTION_STATUS_PIN		GPIO_NUM_0	//0 = bluetooth connected, 1 = bluetooth not connected
#define BLUART_779_CONNECTION_STATUS_PIN		GPIO_NUM_1	// Special case. Not really used. See shared_set_level()


#define BLUART_779_BLUETOOTH_CHANNEL_USED		CHANNEL_2

#define BLUART_779_UART_USED					UART_NUM_1
#define BLUART_779_UART_TX_PIN					GPIO_NUM_10
#define BLUART_779_UART_RX_PIN					GPIO_NUM_9


//////////////////////////////////////////RGB_LED DEFINITIONS//////////////////////////////////////////////
#define RGB_LED1_RED_GPIO		26
#define RGB_LED1_GREEN_GPIO		25
#define RGB_LED1_BLUE_GPIO		14


#endif /* MAIN_E_PUCK_2_H */