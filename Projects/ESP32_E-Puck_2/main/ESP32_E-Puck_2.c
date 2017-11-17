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
#include "rfcomm_E-Puck_2.h"

extern int btstack_main(void);

void main_task(void *pvParameter){
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

void test_send(void *pvParameter){
  uint8_t test_buf[2000];
  uint16_t i = 0;
  uint16_t size = 2000;
  for(i=0;i<size;i++){
    test_buf[i] = i;
  }
  //vTaskDelay(2000 / portTICK_PERIOD_MS);
  // while(1){
  //   vTaskDelay(10 / portTICK_PERIOD_MS);
  //   bluetooth_write(test_buf, size);
  // }

  while(1){
    vTaskDelay(10 / portTICK_PERIOD_MS);
    int16_t rcv = bluetooth_read(test_buf, size);
    //printf("rcv = %d\n",rcv);
    if(rcv > 0){
      // for (i=0;i<rcv;i++){
      //     putchar(test_buf[i]);
      // }
      // printf("\n");
      //bluetooth_write(test_buf,rcv);
    }
  }
}

void app_main(void)
{ 
  xTaskCreate(&main_task, "main_task", 10240, NULL, 5, NULL);
  xTaskCreate(&test_send, "test_send", 5120, NULL, 5, NULL);
  btstack_main();
}
