/*
 * toilet.c
 *
 *  Created on: Feb 2, 2024
 *      Author: SSAFY
 */

#include "toilet.h"
extern uint8_t OCCUPIED_STALL_CNT;
const uint8_t *ESP_MSG_OCCUPIED;


void checkMagnetic(TS stall)
{
    uint8_t current = sensing(stall.magnetic_door);
    if (current == true) {
        if (stall.is_occupied == true) {
            //nothing
        }
        else {
            OCCUPIED_STALL_CNT++;
            stall.is_occupied = true;
//            sendESP(stall.is_occupied, ESP_MSG_OCCUPIED);
        }
    }
    else {
        if (stall.is_occupied == true) {
            OCCUPIED_STALL_CNT--;
            if (stall.last_open_time != -1) {
                if ( (HAL_GetTick() - stall.last_open_time) >= TERM_CHANGE_OCCUPIED) { // GetTick()/(1000*60)%60 ms -> min
                    stall.is_occupied = false;
                    stall.last_open_time = -1;
                    //sendESP(stall.is_occupied, ESP_MSG_OCCUPIED);
                }
            }
            else {
                stall.last_open_time = GetTick();
            }
        }
        else {
            //nothing
        }
    }
    turnLED(stall.led_usable, !current);
}

void untactIR(TS stall) {
    float ir_temperature = getIRTemperature(stall.ir_untact);
    if(ir_temperature >= THRESHOLD_IR_TEMPERATURE) {
        stall.last_ir_time = HAL_GetTick();
        if(stall.is_cover_down == false) {
            runMotor(stall.servo_toilet_cover, MOTOR_TOILET_COVER_PUSH_ANGLE);
            HAL_Delay(MOTOR_DELAY);
            runMotor(stall.servo_toilet_cover, MOTOR_TOILET_COVER_BACK_ANGLE);
        }
    }
}

void flushToilet(TS stall) {
    stall.is_cover_down = sensing(stall.tilt_toilet_cover);
    if (stall.is_cover_down == true) {
        if (HAL_GetTick() - stall.last_ir_time < THRESHOLD_IR_CHECKED_TIME && HAL_GetTick() - stall.last_flush_time < THRESHOLD_FLUSHED_TIME) { // 10s == 10000ms
            /* Flush Toilet
            runMoter(stall.servo_toilet_cover, degree);
            HAL_Delay(MOTOR_DELAY)
            runMoter(stall.servo_toilet_cover, degree);
            */
            stall.last_flush_time = HAL_GetTick();
            stall.is_checked_broken = false;
        }
    }
}

void checkBroken(TS stall) {
    if (stall.is_cover_down) {
        if (WAIT_TOILET_COVER_CLOSED <= HAL_GetTick() - stall.last_flush_time && stall.is_checked_broken == false) { //10s == 10000ms
            stall.is_checked_broken = true;
            runMotor(stall.servo_toilet_cover, MOTOR_CHECK_BROKEN_COVER_PUSH_ANGLE);
            HAL_Delay(MOTOR_DELAY);
            float toilet_water_dist = getDistance(stall.sonar_toilet_broken);
            runMotor(stall.servo_toilet_cover, MOTOR_CHECK_BROKEN_COVER_BACK_ANGLE);
            if (toilet_water_dist < NORMAL_TOILET_WATER_DISTANCE) {
                turnLED(stall.led_broken, true);
//                sendESP(stall., ESP_MSG_OCCUPIED);
            }
            else {
                turnLED(stall.led_broken, false);
            }
        }
    }
}

void checkTissueAmount(TS stall) {
    float current_dist = getDistance(stall.sonar_tissue);
    float tissue_percentage = (MIN_TISSUE_DISTANCE - current_dist) / MAX_TISSUE_RADIUS;

    if (HAL_GetTick() - stall.last_tissue_time > PERIOD_CHECK_TISSUE) { // 1분 == 60만 Tick
//        sendESP(tissue_percentage);
        stall.last_tissue_time = HAL_GetTick();
    }
    if (tissue_percentage <= THRESHOLD_MIN_TISSUE_PERCENTAGE) {
        turnLED(stall.led_tissue, true);
    }
    else {
        turnLED(stall.led_tissue, false);
    }
}

void checkWaterTissue(TS stall)
{
    float distance = Get_Distance(stall.sonar_water_tissue);
    if (distance < THRESHOLD_WATER_TISSUE_HAND_DISTANCE) {
        runMotor(stall.servo_water_tissue, MOTOR_WATER_TISSUE_PUSH_ANGLE);
        HAL_Delay(1000);
        runMotor(stall.servo_water_tissue, MOTOR_WATER_TISSUE_BACK_ANGLE);
    }
}
