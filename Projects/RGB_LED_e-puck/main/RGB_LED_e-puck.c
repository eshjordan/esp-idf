/*

File    : RGB_LED_e-pcuk.c
Author  : Eliot Ferragni
Date    : 18 october 2017
REV 1.0

Fuctions to control the RGB LEDs connected of the ESP32 of the e-puck 2
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "esp_attr.h"   
#include "esp_err.h"
#include "RGB_LED_e-puck.h"

void init_led(void){
  //configure timer for high speed channels
  led_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
  ledc_timer_config(&led_timer);

  //configure timer for low speed channels
  led_timer.speed_mode = LEDC_LOW_SPEED_MODE;
  ledc_timer_config(&led_timer);

  uint8_t rgb_led = 0;
  uint8_t led = 0;
  for(rgb_led = 0 ; rgb_led < NUM_RGB_LED ; rgb_led++){
    for(led = 0 ; led < NUM_LED ; led++){

      //set the configuration
      ledc_channel_config(&led_config[rgb_led][led]);
    }
  }
  //enable the isr
  ledc_fade_func_install(0);
}

void set_led_intensity(rgb_led_name_t rgb_led, led_name_t led, uint8_t intensity, uint16_t time_ms){

  if(intensity > MAX_INTENSITY){
    intensity = MAX_INTENSITY;
  }
  uint16_t value = (100 - intensity) * MAX_DUTY / 100; 
  ledc_set_fade_with_time(led_config[rgb_led][led].speed_mode, 
                          led_config[rgb_led][led].channel, value, time_ms);
  ledc_fade_start(led_config[rgb_led][led].speed_mode, 
                  led_config[rgb_led][led].channel, LEDC_FADE_NO_WAIT);
}

void set_led_color( rgb_led_name_t rgb_led, uint8_t intensity, rgb_color_t* color_value , uint16_t time_ms){

  if(intensity > MAX_INTENSITY){
    intensity = MAX_INTENSITY;
  }

  if(intensity == 0){
    intensity = 1;
  }

  uint16_t value_color[NUM_LED];
  value_color[RED_LED]    = MAX_DUTY - (((((( (color_value->red) * 100 ) / MAX_COLOR_VALUE ) * intensity ) / 100 ) * MAX_DUTY) / 100 ); 
  value_color[GREEN_LED]  = MAX_DUTY - (((((( (color_value->green) * 100 ) / MAX_COLOR_VALUE ) * intensity ) / 100 ) * MAX_DUTY) / 100 );
  value_color[BLUE_LED]   = MAX_DUTY - (((((( (color_value->blue) * 100 ) / MAX_COLOR_VALUE ) * intensity ) / 100 ) * MAX_DUTY) / 100 );

  uint8_t led = 0;
  for(led = 0 ; led < NUM_LED ; led++ ){

    ledc_set_fade_with_time(led_config[rgb_led][led].speed_mode, 
                          led_config[rgb_led][led].channel, value_color[led], time_ms);
    ledc_fade_start(led_config[rgb_led][led].speed_mode, 
                    led_config[rgb_led][led].channel, LEDC_FADE_NO_WAIT);
  }

}

void app_main()
{

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
