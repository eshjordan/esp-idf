/*

File    : RGB_LED_E-Puck.h
Author  : Eliot Ferragni
Date    : 18 october 2017
REV 1.0

Functions to control the RGB LEDs connected of the ESP32 of the E-Puck 2
*/

#ifndef RGB_LED_E_PUCK_H
#define RGB_LED_E_PUCK_H

#include "driver/ledc.h"

#define MAX_DUTY			8191 //based on a 13bits counter for the timer
#define MIN_DUTY			0	
#define PWM_FREQ			5000 // Hz
#define MAX_INTENSITY		100	//percentage
#define MAX_COLOR_VALUE		255	//RGB value

#define LED2_RED_GPIO		32
#define LED2_GREEN_GPIO		33
#define LED2_BLUE_GPIO		25

#define LED4_RED_GPIO		14
#define LED4_GREEN_GPIO		27
#define LED4_BLUE_GPIO		26

#define LED6_RED_GPIO		23
#define LED6_GREEN_GPIO		21
#define LED6_BLUE_GPIO		13

#define LED8_RED_GPIO		4
#define LED8_GREEN_GPIO		17
#define LED8_BLUE_GPIO		15


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


//Some basic color definitons
extern rgb_color_t color[NUM_COLORS];

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


#endif /* RGB_LED_E_PUCK_H */

