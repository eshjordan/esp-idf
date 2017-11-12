/*

File    : RGB_LED_E-Puck.c
Author  : Eliot Ferragni
Date    : 18 october 2017
REV 1.0

Functions to control the RGB LEDs connected of the ESP32 of the E-Puck 2
*/

#include <stdio.h>
//#include "driver/ledc.h"
#include "esp_attr.h"   
#include "esp_err.h"
#include "RGB_LED_E-Puck.h"

//Timer configuration for the LED PWM module
ledc_timer_config_t led_timer = {
    .bit_num = LEDC_TIMER_13_BIT,     //set timer counter bit number
    .freq_hz = PWM_FREQ,                //set frequency of pwm
    .timer_num = LEDC_TIMER_0       //timer index
};

//LED configurations
ledc_channel_config_t led_config[NUM_RGB_LED][NUM_LED] = {
  //LED2
  { 
    //RED_LED
    { 
      .gpio_num     = LED2_RED_GPIO,
      .speed_mode   = LEDC_HIGH_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_0, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0, 
    },
    //GREEN_LED
    { 
      .gpio_num     = LED2_GREEN_GPIO,
      .speed_mode   = LEDC_HIGH_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_1, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0, 
    },
    //BLUE_LED
    { .gpio_num     = LED2_BLUE_GPIO,
      .speed_mode   = LEDC_HIGH_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_2, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0, 
    },
  },
  //LED4
  {
    //RED_LED
    { 
      .gpio_num     = LED4_RED_GPIO,
      .speed_mode   = LEDC_HIGH_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_3, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0, 
    },
    //GREEN_LED
    { 
      .gpio_num     = LED4_GREEN_GPIO,
      .speed_mode   = LEDC_HIGH_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_4,
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0, 
    },
    //BLUE_LED
    { 
      .gpio_num     = LED4_BLUE_GPIO,
      .speed_mode   = LEDC_HIGH_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_5, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0, 
    },
  },
  //LED6
  {
    //RED_LED
    {
      .gpio_num     = LED6_RED_GPIO,
      .speed_mode   = LEDC_HIGH_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_6, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0, 
    },
    //GREEN_LED
    {
      .gpio_num     = LED6_GREEN_GPIO,
      .speed_mode   = LEDC_HIGH_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_7, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0, 
    },
    //BLUE_LED
    {
      .gpio_num     = LED6_BLUE_GPIO,
      .speed_mode   = LEDC_LOW_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_0, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0,       
    },
  },
  //LED8
  {
    //RED_LED
    {
      .gpio_num     = LED8_RED_GPIO,
      .speed_mode   = LEDC_LOW_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_1, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0,       
    },
    //GREEN_LED
    {
      .gpio_num     = LED8_GREEN_GPIO,
      .speed_mode   = LEDC_LOW_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_2, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0,       
    },
    //BLUE_LED
    {
      .gpio_num     = LED8_BLUE_GPIO,
      .speed_mode   = LEDC_LOW_SPEED_MODE,
      .duty         = MAX_DUTY,
      .channel      = LEDC_CHANNEL_3, 
      .intr_type    = LEDC_INTR_FADE_END,
      .timer_sel    = LEDC_TIMER_0,       
    },
  },
};

//Some basic color definitons
rgb_color_t color[NUM_COLORS] ={
  //RED
  {
    .red  = 255,
    .green  = 0,
    .blue   = 0,
  },
  //GREEN
  {
    .red  = 0,
    .green  = 255,
    .blue   = 0,
  },
  //BLUE
  {
    .red  = 0,
    .green  = 0,
    .blue   = 255,
  },
  //YELLOW
  {
    .red  = 255,
    .green  = 255,
    .blue   = 0,
  },
  //LIGHT_BLUE
  {
    .red  = 0,
    .green  = 255,
    .blue   = 255,
  },
  //MAGENTA
  {
    .red  = 255,
    .green  = 0,
    .blue   = 255,
  },
  //WHITE
  {
    .red  = 255,
    .green  = 255,
    .blue   = 255,
  },
};

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

