/*
 * LED.h
 *
 *  Created on: Jan 25, 2024
 *      Author: Kim YR
 */

#ifndef USERLIB_INC_LED_H_
#define USERLIB_INC_LED_H_

#include "stm32f1xx_hal.h"

#define GREEN 0
#define RED 1
#define YELLOW 2
#define BLUE 3



typedef struct LED_t {
	uint16_t port;
	uint pin;
}LED;




#endif /* USERLIB_INC_LED_H_ */
