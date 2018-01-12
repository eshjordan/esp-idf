/*

File    : spi_e-puck2.h
Author  : Stefano Morgani
Date    : 10 January 2018
REV 1.0

Functions to configure and use the SPI communication between the main processor (F407) and the ESP32. 
*/

#ifndef SPI_E_PUCK_2_H
#define SPI_E_PUCK_2_H

#define BUF_SIZE (1024)

#define SPI_TASK_STACK_SIZE	2048
#define SPI_TASK_PRIO		5

/**
 * @brief 	SPI communication handling between the F407 and ESP32
 *
 * @param *pvParameter	parameter from the xCreateTask 	
 */
void spi_task(void *pvParameter);

/**
 * @brief 	SPI initialization (VSPI port).
 *
 */
void spi_init(void);

#endif /* SPI_E_PUCK_2_H */