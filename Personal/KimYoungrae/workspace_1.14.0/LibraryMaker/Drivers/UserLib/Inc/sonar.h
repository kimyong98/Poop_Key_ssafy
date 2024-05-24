/*
 * sonar.h
 *
 *  Created on: Jan 25, 2024
 *      Author: Kim YR
 */

#ifndef USERLIB_INC_SONAR_H_
#define USERLIB_INC_SONAR_H_

#include "stm32f1xx_hal.h"
// #include "stm32f1xx_hal_tim.h"

#define SONAR_WATER 0
#define SONAR_TISSUE 1
#define SONAR_COVER 2

typedef struct GPIO_PIN_T{
	uint16_t trig;
	uint16_t echo;
}GPIO_PIN;



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
uint32_t GetMicroSec(void);
void sonar_timer_init(void);
float get_distance(uint8_t select);

#endif /* USERLIB_INC_SONAR_H_ */







