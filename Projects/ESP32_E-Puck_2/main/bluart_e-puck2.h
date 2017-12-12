/*

File    : bluart_e-puck2.h
Author  : Eliot Ferragni
Date    : 6 december 2017
REV 1.0

Functions to init and use a bluetooth-UART translation
*/

#ifndef BLUART_E_PUCK_2_H
#define BLUART_E_PUCK_2_H

#define BLUART_BUFFER_SIZE 					2000
#define BLUART_UART_BUFFER_SIZE				2000 

#define BLUART_UART_BAUDRATE				230400
#define BLUART_UART_DATABITS				UART_DATA_8_BITS
#define BLUART_UART_PARITY					UART_PARITY_DISABLE
#define BLUART_UART_STOP_BITS				UART_STOP_BITS_1
#define BLUART_UART_FLOWCTRL				UART_HW_FLOWCTRL_DISABLE
#define BLUART_UART_FLOWCTRL_THRESHOLD		122


/*
 *	Init the uart used for the bluart tasks and create the tasks
*/
void bluart_init(void);


#endif /* BLUART_E_PUCK_2_H */