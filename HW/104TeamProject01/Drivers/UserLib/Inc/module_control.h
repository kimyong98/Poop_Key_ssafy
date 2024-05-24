/*
 * module_contorl.h
 *
 *  Created on: Feb 2, 2024
 *      Author: SSAFY
 */

#ifndef USERLIB_INC_MODULE_CONTROL_H_
#define USERLIB_INC_MODULE_CONTROL_H_

#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_tim.h"


// ---------------------------------
#include "global_define.h"

// ---------------------------------



typedef struct GPIOInfo {
    GPIO_TypeDef* Port;
    uint16_t PIN;
    uint16_t PIN_out;
}GI;


typedef struct TIMInfo {
    TIM_HandleTypeDef htim;
    uint32_t channel; // 이거 원래 #define문이라 이슈 가능성 있어요.
}TI;

typedef struct I2CInfo {
    I2C_HandleTypeDef hi2c1;
}II;





void turnLED(GI led, uint8_t onOff);
void motor_start(TI motor);
void runMotor(TI motor, uint16_t degree);
float getDistance(GI sonar);
//void sendESP(uint8_t target, char *msg);
float getIRTemperature(II ir);
uint8_t sensing(GI sensor);



#endif /* USERLIB_INC_MODULE_CONTROL_H_ */

