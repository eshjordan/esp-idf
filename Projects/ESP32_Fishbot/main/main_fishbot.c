/*

File    : main_fishbot.c
Author  : Eliot Ferragni, Stefano Morgani, Daniel Burnier
Date    : 6 December 2018, 4 february 2022
REV 1.0

Firmware to be run by the ESP32 of the Arduino Nano 33 Iot on the Fishbot V5.2
*/

#define __BTSTACK_FILE__ "ESP32_Fishbot.c"

#include <stdio.h>
#include "driver/gpio.h"
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

#include "project_infos.h"

// #include "git_comit.h"
// Find a solution to be able to obtain the project name and the git commit in order to display these infos
// A solution is to pass by the makefile in order to have something like that:
//
//      git_comit.h: FORCE
//      $(Q)echo " GIT git_comit.h"
//      $(Q)echo "#define FIMWARE_INFO $PROJECT_NAME\"$(shell git describe --always --dirty)\"" > $@
//      main_fishbot.c: git_comit.h
//
// For the moment use internal static define

// #define FIMWARE_INFO "ESP32_Fishbot"

extern int btstack_main(void);

void app_main(void)
{ 

  gpio_config_t io_conf;
  //interrupt of falling edge
  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;//GPIO_PIN_INTR_NEGEDGE;
  //bit mask of the pins
  io_conf.pin_bit_mask = ((uint64_t)1 << GPIO_A6);
  //set as input mode    
  io_conf.mode = GPIO_MODE_OUTPUT;
  //enable pull-up mode (no pull-up on pin 34 to 39)
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  //disable pull-down mode (no pull-down on pin 34 to 39)
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io_conf);

  io_conf.intr_type = GPIO_PIN_INTR_DISABLE;//GPIO_PIN_INTR_NEGEDGE;
  //bit mask of the pins
  io_conf.pin_bit_mask = ((uint64_t)1 << GPIO_A7);
  //set as input mode    
  io_conf.mode = GPIO_MODE_OUTPUT;
  //enable pull-up mode (no pull-up on pin 34 to 39)
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  //disable pull-down mode (no pull-down on pin 34 to 39)
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io_conf);

  int gpio_level = 0;

  gpio_set_level(GPIO_A6, gpio_level);
  gpio_set_level(GPIO_A7, gpio_level);

  // rgb_init();
  bluart_init();
  printf("bluart_init launched\n");

  //uart_init();
  // spi_init();

  //a bluetooth echo example
  //Due to a very strange bug of freeRTOS implementation in the ESP32 environment, the tasks related to the bluetooth
  //generate errors in the handling of the semaphores when freeRTOS can choose on which core to execute them.
  //If we specifiy the core for the tasks, it works, no matter which core is chosen for each task.
  // xTaskCreatePinnedToCore(&example_echo_bluetooth_task_channel_1, "example_echo_bluetooth_task", 
  //             EXAMPLE_ECHO_STACK_SIZE, NULL, EXAMPLE_ECHO_PRIO, NULL, CORE_1);
  xTaskCreatePinnedToCore(&example_echo_bluetooth_task_channel_2, "example_echo_bluetooth_task2", 
              EXAMPLE_ECHO_STACK_SIZE, NULL, EXAMPLE_ECHO_PRIO, NULL, CORE_1);
  printf("example_echo_bluetooth_task_channel_2 launched\n");
  xTaskCreatePinnedToCore(&example_echo_bluetooth_task_channel_3, "example_echo_bluetooth_task3", 
              EXAMPLE_ECHO_STACK_SIZE, NULL, EXAMPLE_ECHO_PRIO, NULL, CORE_1);
  printf("example_echo_bluetooth_task_channel_3 launched\n");
  
  //A uart read/write example without event queue;
  //xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
  
  // SPI communication task.
  // xTaskCreatePinnedToCore(spi_task, "spi_task", SPI_TASK_STACK_SIZE, NULL, SPI_TASK_PRIO, NULL, CORE_1);

  printf("\n\nFirmware: ");
  printf(FIMWARE_INFO);
  printf("\n\n");

  //btstack works as a loop called from the main. So every other task should be created before the call
  //of this function
  //main runs always on core 0
  btstack_main();
  printf("This message must NEVER been diplayed\n");
}
