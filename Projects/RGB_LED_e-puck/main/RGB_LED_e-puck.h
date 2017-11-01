/*

File    : RGB_LED_e-pcuk.h
Author  : Eliot Ferragni
Date    : 18 october 2017
REV 1.0

Fuctions to control the RGB LEDs connected of the ESP32 of the e-puck 2
*/

#include "driver/ledc.h"

#define MAX_DUTY			8191 //based on a 13bits counter for the timer
#define MIN_DUTY			0	
#define PWM_FREQ			5000 // Hz
#define MAX_INTENSITY		100	//percentage
#define MAX_COLOR_VALUE		255	//RGB value

//List of the RGB LEDs present on the e-puck 2
typedef enum {
	LED2,
	LED4,
	LED6,
	LED8,
	NUM_RGB_LED,
} rgb_led_name_t;

//List of the LEDs present on each RGB LED
typedef enum {
	RED_LED,
	GREEN_LED,
	BLUE_LED,
	NUM_LED,
} led_name_t;

//List of color examples
typedef enum {
	RED,
	GREEN,
	BLUE,
	YELLOW,
	LIGHT_BLUE,
	MAGENTA,
	WHITE,
	NUM_COLORS,
}color_t;

//struct to represent a color in RGB format (0-255)
typedef struct {
	uint8_t red;
	uint8_t green;
	uint8_t blue;
} rgb_color_t;


//Timer configuration for the LED PWM module
ledc_timer_config_t led_timer = {
    .bit_num = LEDC_TIMER_13_BIT, 		//set timer counter bit number
    .freq_hz = PWM_FREQ,              	//set frequency of pwm
    .timer_num = LEDC_TIMER_0    		//timer index
};

//LED configurations
ledc_channel_config_t led_config[NUM_RGB_LED][NUM_LED] = {
	//LED2
	{	
		//RED_LED
		{	
			.gpio_num		= 32,
			.speed_mode 	= LEDC_HIGH_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_0, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 
		},
		//GREEN_LED
		{	
			.gpio_num		= 33,
			.speed_mode 	= LEDC_HIGH_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_1, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 
		},
		//BLUE_LED
		{	.gpio_num		= 25,
			.speed_mode 	= LEDC_HIGH_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_2, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 
		},
	},
	//LED4
	{
		//RED_LED
		{	
			.gpio_num		= 14,
			.speed_mode 	= LEDC_HIGH_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_3, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 
		},
		//GREEN_LED
		{	
			.gpio_num		= 27,
			.speed_mode 	= LEDC_HIGH_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_4,
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 
		},
		//BLUE_LED
		{	
			.gpio_num		= 26,
			.speed_mode 	= LEDC_HIGH_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_5, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 
		},
	},
	//LED6
	{
		//RED_LED
		{
			.gpio_num		= 22,
			.speed_mode 	= LEDC_HIGH_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_6, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 
		},
		//GREEN_LED
		{
			.gpio_num		= 21,
			.speed_mode 	= LEDC_HIGH_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_7, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 
		},
		//BLUE_LED
		{
			.gpio_num		= 13,
			.speed_mode 	= LEDC_LOW_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_0, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 			
		},
	},
	//LED8
	{
		//RED_LED
		{
			.gpio_num		= 4,
			.speed_mode 	= LEDC_LOW_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_1, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 			
		},
		//GREEN_LED
		{
			.gpio_num		= 17,
			.speed_mode 	= LEDC_LOW_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_2, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 			
		},
		//BLUE_LED
		{
			.gpio_num		= 15,
			.speed_mode 	= LEDC_LOW_SPEED_MODE,
			.duty 			= MAX_DUTY,
			.channel 		= LEDC_CHANNEL_3, 
			.intr_type 		= LEDC_INTR_FADE_END,
			.timer_sel 		= LEDC_TIMER_0, 			
		},
	},
};

//Some basic color definitons
rgb_color_t color[NUM_COLORS] ={
	//RED
	{
		.red 	= 255,
		.green 	= 0,
		.blue 	= 0,
	},
	//GREEN
	{
		.red 	= 0,
		.green 	= 255,
		.blue 	= 0,
	},
	//BLUE
	{
		.red 	= 0,
		.green 	= 0,
		.blue 	= 255,
	},
	//YELLOW
	{
		.red 	= 255,
		.green 	= 255,
		.blue 	= 0,
	},
	//LIGHT_BLUE
	{
		.red 	= 0,
		.green 	= 255,
		.blue 	= 255,
	},
	//MAGENTA
	{
		.red 	= 255,
		.green 	= 0,
		.blue 	= 255,
	},
	//WHITE
	{
		.red 	= 255,
		.green 	= 255,
		.blue 	= 255,
	},
};

/**
 * @brief INIT LED
 *        Init the led and, PWM and timers used to control the leds
 *
 */
void init_led(void);

/**
 * @brief SET LED Intensity
 *        Set the intensity of a specific color of a LED (EACH LED has 3 color intensities)
 *
 * @param rgb_led 	See rgb_led_name_t
 * @param led	 	See led_name_t
 * @param intensity Value between 0 and 100. 100 is completely ON and 0 is completely OFF
 * @param time_ms	Time in ms to take to change the intensity. The intensity will progressively 
 * 					change to the specified value during time_ms (use interruption to work)
 * 					
 */
void set_led_intensity(rgb_led_name_t rgb_led, led_name_t led, uint8_t intensity, uint16_t time_ms);

/**
 * @brief SET LED color
 *        Set the intensity of a specific color of a LED (EACH LED has 3 color intensities)
 *
 * @param led 			See led_name_t
 * @param intensity 	Value between 0 and 100. 100 is completely ON and 0 is completely OFF
 * @param color_value   Struct containing color values in RGB format (0-255) See rgb_color_t
 * @param time_ms		Time in ms to take to change the intensities. The intensities will progressively 
 * 						change to the specified value during time_ms (use interruption to work)
 * 					
 */
void set_led_color(rgb_led_name_t led, uint8_t intensity, rgb_color_t* color_value, uint16_t time_ms);




