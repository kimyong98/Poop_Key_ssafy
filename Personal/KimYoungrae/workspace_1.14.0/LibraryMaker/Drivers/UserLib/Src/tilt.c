/*
 * tilt.c
 *
 *  Created on: Jan 25, 2024
 *      Author: Kim YR
 */




#include "tilt.h"

uint8_t is_tilt_horizon(void){
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == 0) {
		return HAL_HORIZON;
	}
	else {
		return HAL_VERTICAL;
	}
}
