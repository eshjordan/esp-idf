/*

File    : socket_e-puck2.h
Author  : Stefano Morgani
Date    : 22 March 2018
REV 1.0

Functions to configure and use the socket to exchange data through WiFi.
*/

#ifndef SOCKET_E_PUCK_2_H
#define SOCKET_E_PUCK_2_H

#define BUF_SIZE (1024)

#define SOCKET_TASK_STACK_SIZE	2048
#define SOCKET_TASK_PRIO		5

/**
 * @brief 	SPI communication handling between the F407 and ESP32
 *
 * @param *pvParameter	parameter from the xCreateTask 	
 */
void socket_task(void *pvParameter);

/**
 * @brief 	SPI initialization (VSPI port).
 *
 */
void socket_init(void);

void socket_set_event_connected(void);
void socket_set_event_disconnected(void);
void socket_set_event_data_ready(void);

#endif /* SOCKET_E_PUCK_2_H */
