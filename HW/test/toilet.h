/*
 * toilet.h
 *
 *  Created on: Feb 2, 2024
 *      Author: SSAFY
 */

#ifndef USERLIB_INC_TOILET_H_
#define USERLIB_INC_TOILET_H_
#include "stm32f1xx_hal.h"
#include "module_control.h"

#define true 1
#define false 0


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


typedef struct ToiletStall {
    uint8_t is_occupied; // 원래 bool
    uint32_t last_open_time; // 원래 int
    float tissue_amount;
    uint8_t is_cover_down;  // 원래 bool
    uint32_t last_ir_time;  // 원래 int
    uint32_t last_flush_time;  // 원래 int
    uint8_t is_checked_broken; // 원래 bool
    uint32_t last_tissue_time; // 원래 int

    GI magnetic_door;
    GI sonar_tissue;
    GI sonar_water_tissue;
    GI sonar_toilet_broken;
    II ir_untact;
    GI led_usable;
    GI led_broken;
    GI led_tissue;
    TI servo_water_tissue;
    TI servo_toilet_cover;
    GI tilt_toilet_cover;
}TS;


#endif /* USERLIB_INC_TOILET_H_ */
