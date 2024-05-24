/*
 * module_control.c
 *
 *  Created on: Feb 2, 2024
 *      Author: SSAFY
 */

#include "module_control.h"
//#include "stm32f1xx_hal_tim.h"


//extern void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//extern uint32_t GetMicroSec(void);

uint32_t overflows = 0U;
extern TIM_HandleTypeDef htim1;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { // 제네레이트 없음
	if(htim->Instance == TIM1) {
		overflows++;
	}
}


uint32_t GetMicroSec(void){ // 제네레이트 없음
	uint32_t count = __HAL_TIM_GET_COUNTER(&htim1);
	uint32_t overflow = overflows;
	if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) && (count < 0x8000)) {
	        overflow++;
	}

	return(overflow << 16) + count;
}


uint8_t sensing(GI sensor) { // 마그네틱 같은거
    uint8_t ret = HAL_GPIO_ReadPin(sensor.Port, sensor.PIN);
    return ret;
}

void turnLED(GI led, uint8_t is_ON) { // bool 을 uint8_t로 바꾸었습니다.
    HAL_GPIO_WritePin(led.Port, led.PIN_out, is_ON);
}


void motor_start(TI motor) { // 우리는 모터가 두 개라서 두개를 스타트 해야합니다.
    HAL_TIM_PWM_Start(&(motor.htim), motor.channel);
}

void runMotor(TI motor, uint16_t degree) { // 근데 이거 조금 생각해봐야 할거에요. g
    __HAL_TIM_SET_COMPARE(&(motor.htim), motor.channel, degree);

    // degree는 500이 0도 // 1000 이 90도
    // main문을 하면서 다시 검증해볼 필요 있음 (MG996R이랑 SG90이랑 Duty 같음)
}

float getDistance(GI Sensor) { // 초음파
    HAL_GPIO_WritePin(Sensor.Port, Sensor.PIN_out, RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(Sensor.Port, Sensor.PIN_out, SET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(Sensor.Port, Sensor.PIN_out, RESET);

    //printf("right after : %lu\r\n", HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9));

    uint32_t start_time = HAL_GetTick();
    while (HAL_GPIO_ReadPin(Sensor.Port, Sensor.PIN) == GPIO_PIN_RESET){
    	if(HAL_GetTick() - start_time > 1000){
    		return -1.0;
    	}
    }
    uint32_t st = GetMicroSec();
    start_time = HAL_GetTick();
    while (HAL_GPIO_ReadPin(Sensor.Port, Sensor.PIN) == GPIO_PIN_SET){
    	if(HAL_GetTick() - start_time > 1000){
			return -1.0;
    	}
    }
    uint32_t ed = GetMicroSec();

    uint32_t diff = ed - st;
    float ret = diff * 0.034 / 2;

    return ret;
}

float getIRTemperature(II ir) {
    // 이건 "mlx90614.h" 와 "mlx90614.c" 가 있어용.
    // 복사해서 쓰면 함수 쓸 수 있어용.
    // 함수 : float MLX90614_ReadTemperature(void);
    // 앤 그외에 적을거 없어요.

    uint8_t data[3] = { 0, };
    HAL_I2C_Mem_Read(&(ir.hi2c1), (0x5A << 1), 0x07, 0x00000001U, data, 3, HAL_MAX_DELAY);
    //HAL_I2C_Mem_Read(&hi2c1, MLX90614_I2C_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, data, 3, HAL_MAX_DELAY);

    int16_t rawTemperature = (data[1] << 8) | data[0];
    float temperature = rawTemperature * 0.02 - 273.15;

    return temperature;
}
