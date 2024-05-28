/*
 * module_contorl.h
 *
 *  Created on: Feb 2, 2024
 *      Author: SSAFY
 */

#ifndef USERLIB_INC_MODULE_CONTROL_H_
#define USERLIB_INC_MODULE_CONTROL_H_

#include "stm32f103xb.h"
#include "stm32f1xx_hal_tim.h"


// ---------------------------------
#define MAX_TISSUE_RADIUS 0
#define MIN_TISSUE_DISTANCE 0
#define NORMAL_TOILET_WATER_DISTANCE 0
#define THRESHOLD_MIN_TISSUE_PERCENTAGE 0
#define THRESHOLD_WATER_TISSUE_HAND_DISTANCE 0
#define THRESHOLD_IR_TEMPERATURE 0
#define THRESHOLD_FLUSHED_TIME 0
#define THRESHOLD_IR_CHECKED_TIME 0
#define TERM_CHANGE_OCCUPIED 0
#define WAIT_TOILET_COVER_CLOSED 0
#define MOTOR_WATER_TISSUE_PUSH_ANGLE 0
#define MOTOR_WATER_TISSUE_BACK_ANGLE 0
#define MOTOR_TOILET_COVER_PUSH_ANGLE 0
#define MOTOR_TOILET_COVER_BACK_ANGLE 0
#define MOTOR_CHECK_BROKEN_COVER_PUSH_ANGLE 0
#define MOTOR_CHECK_BROKEN_COVER_BACK_ANGLE 0
#define MOTOR_CHECK_BROKEN_COVER_DELAY 0
#define MOTOR_DELAY 0
#define PERIOD_CHECK_TISSUE 0
#define ESP_MUX_ID_RASPI 0
#define ESP_MUX_ID_BACKEND 0

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
void runMotor(TI motor, int degree);
float getDistance(GI sonar);
void sendESP(uint8_t target, char *msg);
float getIRTemperature(II ir);
uint8_t sensing(GI sensor);



#endif /* USERLIB_INC_MODULE_CONTROL_H_ */

