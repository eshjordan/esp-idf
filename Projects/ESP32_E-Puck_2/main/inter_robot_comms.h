/*

File    : inter_robot_comms.h
Author  : Jordan Esh
Date    : 19 September 2024
REV 1.0

Functions to configure and use the inter-robot comms to exchange data through WiFi.
*/

#ifndef INTER_ROBOT_COMMS_H
#define INTER_ROBOT_COMMS_H

#define ACTUATORS_BUFF_LEN 21

#define INTER_ROBOT_COMMS_TASK_STACK_SIZE	2048
#define INTER_ROBOT_COMMS_TASK_PRIO		5

/**
 * @brief 	TCP communication handling between the PC and ESP32
 *
 * @param *pvParameter	parameter from the xCreateTask
 */
void inter_robot_comms_task(void *pvParameter);

/**
 * @brief 	Inter-robot comms initialization.
 *
 */
void inter_robot_comms_init(void);

/**
 * @brief 	Set the connected bit.
 *
 */
void inter_robot_comms_set_event_connected(void);

/**
 * @brief 	Set the disconnected bit.
 *
 */
void inter_robot_comms_set_event_disconnected(void);

#endif /* INTER_ROBOT_COMMS_H */
