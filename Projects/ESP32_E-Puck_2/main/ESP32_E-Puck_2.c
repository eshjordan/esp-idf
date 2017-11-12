/*

File    : ESP32_E-Puck_2.c
Author  : Eliot Ferragni
Date    : 12 november 2017
REV 1.0

Firmware to be run on the ESP32 of the E-Puck 2
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "esp_attr.h"   
#include "esp_err.h"
#include "RGB_LED_E-Puck.h"
#include "UART_E-Puck_2.h"

void app_main(){
  //A uart read/write example without event queue;
  xTaskCreate(echo_task, "uart_echo_task", 1024, NULL, 10, NULL);
  
  init_led();

  rgb_color_t color_value;
  uint8_t intensity = 100;

  uint8_t color_nb = 0;
  uint8_t rgb_led = 0;
  while(1){

    for(color_nb = 0 ; color_nb < NUM_COLORS ; color_nb++){
      color_value = color[color_nb];
      for(rgb_led = 0 ; rgb_led < NUM_RGB_LED ; rgb_led++){
        set_led_color(rgb_led, intensity, &color_value, 5);
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
    }
  }
}
