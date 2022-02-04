/*

File    : button_fishbot.h
Author  : Eliot Ferragni, Daniel Burnier
Date    : 6 december 2017, 4 february 2022
REV 1.0

Functions to configure and use the button through interruption
*/


#ifndef BUTTON_FISHBOT_H
#define BUTTON_FISHBOT_H

/**
 * @brief return if the button is pressed or not
 *
 * @return		1 if pressed and 0 if not
 */
uint8_t button_is_pressed(void);

/**
 * @brief Init the gpio of the button and the ISR related
 */
void button_init(void);


#endif /* BUTTON_FISHBOT_H */