/*
 * hack_led.c
 *
 *  Created on: Aug 25, 2022
 *      Author: Gerlie
 */


#include "hack_led.h"
#include <math.h>

char count = 0;

static inline void gpio_rising(hack_led_gpio gpio){
	HAL_GPIO_WritePin(gpio.gpio, gpio.pin_num, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(gpio.gpio, gpio.pin_num, GPIO_PIN_SET);
	HAL_Delay(50);
}

void init_hack_led(hack_led_t* hack_led, GPIO_TypeDef* gpio, uint16_t pin_num){
	hack_led->current_state = TURN_OFF;
	hack_led->gpio.gpio = gpio;
	hack_led->gpio.pin_num = pin_num;
	HAL_GPIO_WritePin(gpio, pin_num, GPIO_PIN_SET);
}

_Bool hack_led_off(hack_led_t* hack_led){
//	hack_led_state state = hack_led->current_state;
//	for(; state != 0; state = (state + 1) % 5)
//		gpio_rising(hack_led->gpio);
//	hack_led->current_state = state;
	return hack_led_set(hack_led, TURN_OFF);
}

//_Bool hack_led_set(hack_led_t* hack_led, hack_led_state state){
//	hack_led_state current_state = hack_led->current_state;
//	if(current_state == state)
//		return hack_led_false;
//
//	for(; current_state != state; current_state = (current_state + 1) % 5)
//		gpio_rising(hack_led->gpio);
//	hack_led->current_state = current_state;
//	return hack_led_true;
//}

_Bool hack_led_set(hack_led_t* hack_led, hack_led_state state){
	hack_led_state current_state = hack_led->current_state;
	if(current_state == state) return hack_led_false;
	count = (current_state > state) ? (state + 5) - current_state : state - current_state;
	count *= 2; // For Rising Edge
	hack_led->current_state = state;
	return hack_led_true;
}

void hack_led_gpio_set_reset(hack_led_t* hack_led, GPIO_PinState pin_state){
	HAL_GPIO_WritePin(hack_led->gpio.gpio, hack_led->gpio.pin_num, pin_state);
}
