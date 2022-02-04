/*

File    : main_fishbot.c
Author  : Eliot Ferragni, Stefano Morgani, Daniel Burnier
Date    : 6 December 2018, 4 february 2022
REV 1.0

Firmware to be run by the ESP32 of the Arduino Nano 33 Iot on the Fishbot V5.2
*/

#define __BTSTACK_FILE__ "ESP32_Fishbot.c"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "esp_attr.h"   
#include "esp_err.h"
#include "main_fishbot.h"
#include "rgb_led_fishbot.h"
#include "uart_fishbot.h"
#include "bluart_fishbot.h"
#include "rfcomm_fishbot.h"
#include "spi_fishbot.h"

extern int btstack_main(void);

void app_main(void)
{ 
  rgb_init();
  bluart_init();
  //uart_init();
  spi_init();

  //a bluetooth echo example
  //Due to a very strange bug of freeRTOS implementation in the ESP32 environment, the tasks related to the bluetooth
  //generate errors in the handling of the semaphores when freeRTOS can choose on which core to execute them.
  //If we specifiy the core for the tasks, it works, no matter which core is chosen for each task.
  // xTaskCreatePinnedToCore(&example_echo_bluetooth_task_channel_1, "example_echo_bluetooth_task", 
  //             EXAMPLE_ECHO_STACK_SIZE, NULL, EXAMPLE_ECHO_PRIO, NULL, CORE_1);
  // xTaskCreatePinnedToCore(&example_echo_bluetooth_task_channel_2, "example_echo_bluetooth_task2", 
  //             EXAMPLE_ECHO_STACK_SIZE, NULL, EXAMPLE_ECHO_PRIO, NULL, CORE_1);
  xTaskCreatePinnedToCore(&example_echo_bluetooth_task_channel_3, "example_echo_bluetooth_task3", 
              EXAMPLE_ECHO_STACK_SIZE, NULL, EXAMPLE_ECHO_PRIO, NULL, CORE_1);
  
  //A uart read/write example without event queue;
  //xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
  
  // SPI communication task.
  xTaskCreatePinnedToCore(spi_task, "spi_task", SPI_TASK_STACK_SIZE, NULL, SPI_TASK_PRIO, NULL, CORE_1);

  //btstack works as a loop called from the main. So every other task should be created before the call
  //of this function
  //main runs always on core 0
  btstack_main();
}
