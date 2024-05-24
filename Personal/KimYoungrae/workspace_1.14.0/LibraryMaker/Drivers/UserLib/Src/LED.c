/*
 * LED.c
 *
 *  Created on: Jan 25, 2024
 *      Author: Kim YR
 */


#include "LED.h"

LED led[3][4] = { // [toilet kan][color select - GREEN RED YELLOW BLUE]
		{{GPIOA, GPIO_PIN_1},{GPIOA, GPIO_PIN_1},{GPIOA, GPIO_PIN_1},{GPIOA, GPIO_PIN_1}},
		{{GPIOA, GPIO_PIN_1},{GPIOA, GPIO_PIN_1},{GPIOA, GPIO_PIN_1},{GPIOA, GPIO_PIN_1}},
		{{GPIOA, GPIO_PIN_1},{GPIOA, GPIO_PIN_1},{GPIOA, GPIO_PIN_1},{GPIOA, GPIO_PIN_1}}
};



uint8_t led_change_color(uint8_t toilet_select, uint8_t color) {
	if(color == GREEN) {


		return 0;
	}
	else if(color == RED) {

		return 0;
	}
	else if(color == YELLOW) {

		return 0;
	}
	else if(color == BLUE) {

		return 0;
	}
	else
		return 1;

}
