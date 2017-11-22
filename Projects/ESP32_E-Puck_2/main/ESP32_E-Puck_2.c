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
#include "RGB_LED_E-Puck.h"
#include "UART_E-Puck_2.h"
#include "rfcomm_E-Puck_2.h"
#include "button_e-puck2.h"

extern int btstack_main(void);

void app_main(void)
{ 
  init_led();
  button_init();

  xTaskCreate(&example_echo_bluetooth_task, "example_echo_bluetooth_task", 5120, NULL, 5, NULL);
  //A uart read/write example without event queue;
  xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
  btstack_main();
}
