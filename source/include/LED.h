/*
 * LED.h
 *
 *  Created on: 18. 11. 2018
 *      Author: jakub
 */

#ifndef LED_H_
#define LED_H_
#include <fsl_gpio.h>
#include <pin_mux.h>

typedef enum color{
	RED = BOARD_INITPINS_LED_RED_PIN,
	GREEN = BOARD_INITPINS_LED_GREEN_PIN,
	BLUE = BOARD_INITPINS_LED_BLUE_PIN
}COLOR;

inline void LED_turnOff(COLOR farba){
	GPIO_WritePinOutput(farba == BLUE ? GPIOD : GPIOB, farba, 1);
//	}
}

inline void LED_turnOn(COLOR farba){
	GPIO_WritePinOutput(farba == BLUE ? GPIOD : GPIOB, farba, 0);
}

inline void LED_switch(COLOR farba){
	GPIO_TogglePinsOutput(farba == BLUE ? GPIOD : GPIOB, farba);
}
#endif /* LED_H_ */
