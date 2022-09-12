/*
 * hack_led.h
 *
 *  Created on: Aug 25, 2022
 *      Author: Gerlie
 */

#ifndef INC_HACK_LED_H_
#define INC_HACK_LED_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define hack_led_true  (_Bool)1u
#define hack_led_false (_Bool)0u

typedef enum __attribute__((packed)){
	TURN_OFF = 0,  // TURN OFF
	TURN_ON_FRONT, // Only Front light
	TURN_ON_1,     // Normal Light
	TURN_ON_2,     // Twinkle
	TURN_ON_3,     // Twinkle Twinkle
}hack_led_state;

typedef struct {
	GPIO_TypeDef* gpio;
	uint16_t pin_num;
}hack_led_gpio;

typedef struct {
	hack_led_gpio gpio;
	hack_led_state current_state;
}hack_led_t;

void init_hack_led(hack_led_t* hack_led, GPIO_TypeDef* gpio, uint16_t pin_num);
_Bool hack_led_off(hack_led_t* hack_led);
_Bool hack_led_set(hack_led_t* hack_led, hack_led_state led_state);
void hack_led_gpio_set_reset(hack_led_t* hack_led, GPIO_PinState pin_state);

#endif /* INC_HACK_LED_H_ */
