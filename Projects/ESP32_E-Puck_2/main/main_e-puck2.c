/*

File    : ESP32_E-Puck_2.c
Author  : Eliot Ferragni
Date    : 12 november 2017
REV 1.0

Firmware to be run on the ESP32 of the E-Puck 2
*/

#define __BTSTACK_FILE__ "ESP32_E-Puck_2.c"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "esp_attr.h"   
#include "esp_err.h"
#include "rgb_led_e-puck2.h"
#include "uart_e-puck2.h"
#include "rfcomm_e-puck2.h"
#include "button_e-puck2.h"

extern int btstack_main(void);

void app_main(void)
{ 
  rgb_init();
  button_init();

  //a bluetooth echo example
  xTaskCreate(&example_echo_bluetooth_task, "example_echo_bluetooth_task", 
              EXAMPLE_ECHO_STACK_SIZE, NULL, EXAMPLE_ECHO_PRIO, NULL);
  //A uart read/write example without event queue;
  xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
  //btstack works as a loop called from the main. So every other task should be created befor the call
  //of this function
  btstack_main();
}
