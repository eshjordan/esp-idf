/*

File    : rfcomm_E-Puck_2.c
Author  : Eliot Ferragni
Date    : 17 november 2017
REV 1.2

Functions to comtrol and use the bluetooth stack
*/


#ifndef RFCOMM_E_PUCK2_H
#define RFCOMM_E_PUCK2_H

//#define ENABLE_LOG_RFCOMM

#define BLUE_RX_BUFFER_SIZE		2000	
#define BLUE_TX_BUFFER_SIZE		2000

#define DATAS_WRITTEN			0

#define BUFFER_FULL				-1
#define TASK_COLLIISION			-2
#define BLUETOOTH_NOT_CONNECTED	-3

#ifdef ENABLE_LOG_RFCOMM
#define log_rfcomm(format, ...)  printf(format,  ## __VA_ARGS__)
#else
#define log_rfcomm(...) __log_unused(__VA_ARGS__)
#endif

/**
 * @brief Write datas to the bluetooth
 *        To achieve best performances, it is best to provide a large amount of datas at the same time
 *        and to wait minimum 10ms between two calls of this function.
 *
 * @param buffer 		Pointer to a buffer containing the datas to send
 * @param buffer_len	Size of the buffer provided
 *
 * @return	true if the datas has been taken and false if the buffer is already full
 * 			or the bluetooth not available.
 */
int8_t bluetooth_write(uint8_t* buffer, uint16_t buffer_len);

/**
 * @brief Read datas from the bluetooth
 *        To achieve best performances, it is best to provide a large buffer
 *        in order to read more at the same time and to wait minimum 10ms between two calls of this function
 *
 * @param buffer 		Pointer to a buffer to feed with the received datas
 * @param buffer_len	Size of the buffer provided
 *
 * @return	-1 if the bluetooth is not connected
 * 			0 if nothing has been read
 * 			else the number of datas read
 */
int16_t bluetooth_read(uint8_t* buffer, uint16_t buffer_len);


#endif /* RFCOMM_E_PUCK2_H */