/*
 * toilet.c
 *
 *  Created on: Feb 2, 2024
 *      Author: SSAFY
 */

#include "toilet.h"
// kyr add stdio for printf warning
#include <stdio.h>

const uint8_t TOILET_ID = 1;

const uint8_t SATURATION = 2;
const uint8_t CONFUSION = 1;
const uint8_t NORMAL = 0;

extern volatile uint8_t OCCUPIED_STALL_CNT;
extern volatile uint8_t is_saturation;

// sign
uint8_t MsgBox[MsgSize];

//const uint8_t MSG_OCCUPIED[MsgSize];
//
//const uint8_t MSG_FREE[MsgSize];
//
//const uint8_t MSG_TISSUE_EMPTY[MsgSize];
//
//const uint8_t MSG_BROKEN[MsgSize];





extern I2C_HandleTypeDef hi2c1;

void checkMagnetic(TS* stall)
{
    uint8_t current = sensing(stall->magnetic_door);



    if (current == true) {
        if (stall->is_occupied == true) {
            //nothing
        }
        else {
            OCCUPIED_STALL_CNT++;
            stall->is_occupied = true;
            memset(MsgBox, 0, sizeof(MsgBox));
            sprintf(MsgBox, "toiletOccupied,%u,\r\n", stall->toilet_key);
            SendDataToRasp(strlen((char *)MsgBox), MsgBox);
        }
    }
    else {
        if (stall->is_occupied == true) {
            if (stall->last_open_time != 0) {
                if ( (HAL_GetTick() - stall->last_open_time) >= TERM_CHANGE_OCCUPIED) { // GetTick()/(1000*60)%60 ms -> min
                	OCCUPIED_STALL_CNT--;
                	stall->is_occupied = false;
                    stall->last_open_time = 0;
                    memset(MsgBox, 0, sizeof(MsgBox));
                    sprintf(MsgBox, "toiletVacant,%u,\r\n", stall->toilet_key);
                    SendDataToRasp(strlen((char *)MsgBox), MsgBox);
                }
            }
            else {
                stall->last_open_time = HAL_GetTick();
            }
        }
        else {
            //nothing
        }
    }
    turnLED(stall->led_usable, !current);


#if DEBUG
    printf("magnetic: %u, ", current);
#endif


}

void checkTissueAmount(TS* stall) {
    float current_dist = getDistance(stall->sonar_tissue);
//    float tissue_percentage = (MIN_TISSUE_DISTANCE - current_dist) / MAX_TISSUE_RADIUS * 100;
    float tissue_percentage = ((MIN_TISSUE_DISTANCE - current_dist) / (MIN_TISSUE_DISTANCE - MAX_TISSUE_RADIUS)) * 100;
//    if(tissue_percentage <= 0.0) tissue_percentage = 0.0f;

    // float -> uint8_t
    uint8_t percentage = tissue_percentage;
    memset(MsgBox, 0, sizeof(MsgBox));
    sprintf(MsgBox, "tissueStatus,%u,%u,\r\n", percentage,stall->toilet_key);
    SendDataToRasp(strlen((char *)MsgBox), MsgBox);

    if (HAL_GetTick() - stall->last_tissue_time > PERIOD_CHECK_TISSUE) { // 1분 == 60만 Tick
        stall->last_tissue_time = HAL_GetTick();
    }
    if (tissue_percentage <= THRESHOLD_MIN_TISSUE_PERCENTAGE) {
    		stall->before_tissue_empty = 1;

    	turnLED(stall->led_tissue, true);
    }
    else {
        turnLED(stall->led_tissue, false);
    }
#if DEBUG
    printf("tissue dist: %.3f, percentage: %.3f %%, ", current_dist, tissue_percentage);
#endif
}

void checkWaterTissue(TS* stall)
{
	float distance = getDistance(stall->sonar_water_tissue);
    if (5 < distance && distance < THRESHOLD_WATER_TISSUE_HAND_DISTANCE) {
        runMotor(stall->servo_water_tissue, MOTOR_WATER_TISSUE_PUSH_ANGLE);
        HAL_Delay(1500);
        runMotor(stall->servo_water_tissue, MOTOR_WATER_TISSUE_BACK_ANGLE);
    }
#if DEBUG
    printf("water tissue dist: %.3f, ", distance);
#endif
}



void untactIR(TS* stall) {
    float ir_temperature = getIRTemperature(stall->ir_untact);
    if(ir_temperature >= THRESHOLD_IR_TEMPERATURE && (stall->last_ir_time == 0 || HAL_GetTick() - stall->last_ir_time >= TERM_PUSH_TOILET_COVER)) {
        stall->last_ir_time = HAL_GetTick();
        stall->is_flushed = false;
//        if(stall->is_cover_down == true) {
//            runMotor(stall->servo_toilet_cover, MOTOR_TOILET_COVER_PUSH_ANGLE);
//            HAL_Delay(MOTOR_DELAY);
//            runMotor(stall->servo_toilet_cover, MOTOR_TOILET_COVER_BACK_ANGLE);
//        }
    }
#if DEBUG
	printf("IR : %.3f, ", ir_temperature);
#endif
}

void flushToilet(TS* stall) {
    stall->is_cover_down = sensing(stall->tilt_toilet_cover);
    if (stall->is_cover_down == false && stall->last_ir_time != 0) {
    	// < => >
        if (HAL_GetTick() - stall->last_ir_time < THRESHOLD_IR_CHECKED_TIME && stall->is_flushed == false) { // 10s == 10000ms
            /* Flush Toilet
            runMoter(stall.servo_toilet_cover, degree);
            HAL_Delay(MOTOR_DELAY)
            runMoter(stall.servo_toilet_cover, degree);
            */
            stall->last_flush_time = HAL_GetTick();
            stall->is_checked_broken = false;
            stall->is_flushed = true;
        }
    }
#if DEBUG
    printf("is_cover_down: %u, last_ir_time %lu, last_flush_time %lu, ", stall->is_cover_down, stall->last_ir_time, stall->last_flush_time);
#endif
}

void checkBroken(TS *stall) {
//	runMotor(stall->servo_sonar_cover, MOTOR_CHECK_BROKEN_COVER_PUSH_ANGLE);
//	HAL_Delay(MOTOR_DELAY);
//	HAL_Delay(2000);
//	float toilet_water_dist = getDistance(stall->sonar_toilet_broken);
//	runMotor(stall->servo_sonar_cover, MOTOR_CHECK_BROKEN_COVER_BACK_ANGLE);
//	HAL_Delay(MOTOR_DELAY);
	float toilet_water_dist = -2;
    if (stall->is_cover_down && stall->last_flush_time != 0) {
        if (WAIT_TOILET_FLUSH_DOWN <= HAL_GetTick() - stall->last_flush_time && stall->is_checked_broken == false) { //10s == 10000ms
//            printf("open cover")
        	stall->is_checked_broken = true;
        	// Edit
            runMotor(stall->servo_sonar_cover, MOTOR_CHECK_BROKEN_COVER_PUSH_ANGLE);
            HAL_Delay(MOTOR_DELAY);
            toilet_water_dist = getDistance(stall->sonar_toilet_broken);
            runMotor(stall->servo_sonar_cover, MOTOR_CHECK_BROKEN_COVER_BACK_ANGLE);
            if (toilet_water_dist < NORMAL_TOILET_WATER_DISTANCE) {
                turnLED(stall->led_broken, true);

                memset(MsgBox, 0, sizeof(MsgBox));
                sprintf(MsgBox, "toiletBreak,%u,\r\n", stall->toilet_key);
				SendDataToRasp(strlen((char *)MsgBox), MsgBox);


            }
            else {
                turnLED(stall->led_broken, false);
            }
        }
    }
#if DEBUG
//    printf("Is broken LED ON: %u, ",HAL_GPIO_ReadPin(stall->led_broken.Port, stall->led_broken.PIN_out));
    printf("Is broken LED ON: %u, Water distance: %.3f, ",HAL_GPIO_ReadPin(stall->led_broken.Port, stall->led_broken.PIN_out), toilet_water_dist);
#endif
}

void checkCongest(TS * stall) {
	memset(MsgBox, 0, sizeof(MsgBox));
	uint8_t debugging;
	if(is_saturation == 0) return;
	else if(is_saturation == 2) { // 2
		debugging = SATURATION;
		sprintf(MsgBox,"congestion,%u,%u,", SATURATION, TOILET_ID); // 0 is toilet ID not toilet's cell ID
	}
	else if(is_saturation == 1 && OCCUPIED_STALL_CNT == MAX_STALL) { // 1
		debugging = CONFUSION;
		sprintf(MsgBox,"congestion,%u,%u,", CONFUSION, TOILET_ID);
	}
	else if(is_saturation == 1 && OCCUPIED_STALL_CNT < MAX_STALL) { // 0
		debugging = NORMAL;
		sprintf(MsgBox,"congestion,%u,%u,", NORMAL, TOILET_ID);
	}

	SendDataToRasp(strlen((char *)MsgBox), MsgBox);
#if DEBUG
		printf("Congestion: %u, is_saturation: %u", debugging, is_saturation);
#endif

}
void initStalls(TS * stall) {
	//
	stall->toilet_key = 1;
	stall->before_tissue_empty = 0;


	stall->is_occupied = false;
	stall->last_open_time = 0;
	stall->tissue_amount = 100;
	stall->is_cover_down = false;
	stall->last_ir_time = 0;
	stall->last_flush_time = 0;
	stall->is_checked_broken = false;
	stall->last_tissue_time = 0;


	stall->magnetic_door = (GI){ GPIOB,GPIO_PIN_4, 0 };

	stall->sonar_water_tissue = (GI){ GPIOA, GPIO_PIN_12, GPIO_PIN_11 };
	stall->servo_water_tissue = (TI){ htim2, TIM_CHANNEL_2 };

	stall->sonar_tissue = (GI){ GPIOB, GPIO_PIN_2, GPIO_PIN_1 };

	stall->ir_untact = (II){ hi2c1 };
	stall->servo_toilet_cover = (TI){  htim2, TIM_CHANNEL_1 };


	stall->tilt_toilet_cover = (GI){ GPIOB, GPIO_PIN_5, 0 };
	stall->servo_sonar_cover = (TI){ htim2, TIM_CHANNEL_3 };
	stall->sonar_toilet_broken = (GI){ GPIOB, GPIO_PIN_12, GPIO_PIN_11 };

	stall->led_usable = (GI){ GPIOC, 0, GPIO_PIN_5 };
	stall->led_broken = (GI){ GPIOC, 0, GPIO_PIN_8 };
	stall->led_tissue = (GI){ GPIOC, 0, GPIO_PIN_6 };

	motor_start(stall->servo_water_tissue);
	motor_start(stall->servo_sonar_cover);
	motor_start(stall->servo_toilet_cover);

}

