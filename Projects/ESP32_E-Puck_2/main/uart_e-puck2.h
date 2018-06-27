/*

File    : uart_e-puck2.h
Author  : Stefano Morgani
Date    : 10 January 2018
REV 1.0

Functions to configure and use the UART communication between the ESP32 and both the main processor (F407) and the programmer (F413).
*/

#ifndef UART_E_PUCK_2_H
#define UART_E_PUCK_2_H

#define UART_TX_BUFF_SIZE 28
#define UART_RX_BUFF_SIZE 64

#define ECHO_TASK_STACK_SIZE	1024
#define ECHO_TASK_PRIO			10

typedef enum {
	SENSORS_BUFF_EMPTY,
	SENSORS_BUFF_FILLED
} sensors_buffer_state_t;

typedef struct {
	sensors_buffer_state_t state;
	uint8_t* data;
} sensors_buffer_t;

/**
 * @brief Echo from UART2 (F407) to UART0(F413)/UART2(F407) without flow control.
 *
 * @param *pvParameter	parameter from the xCreateTask 	
 */
void echo_task(void *pvParameter);

/**
 * @brief 	UART initialization.
 *
 */
void uart_init(void);


void uart_set_actuators_state(uint8_t *buff);

sensors_buffer_t *uart_get_data_ptr(void);


#endif /* UART_E_PUCK_2_H */