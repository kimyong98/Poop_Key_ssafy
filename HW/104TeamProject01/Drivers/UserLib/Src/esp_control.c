/*
 * esp_control.c
 *
 *  Created on: Feb 2, 2024
 *      Author: SSAFY
 */


#include <string.h>
#include <stdio.h>
#include "esp_control.h"

uint8_t BBOX[1024] = {0, };
uint8_t buff;
volatile uint8_t BBOX_count = 0;

extern UART_HandleTypeDef huart1;

uint8_t Rx_buffer[1024];
uint16_t Rx_Head = 0;
uint16_t Rx_Tail = 0;

void WriteBuffer(uint8_t buff){
	Rx_buffer[Rx_Head] = buff;
	Rx_Head = (Rx_Head + 1) % UART_RX_BUFFER_SIZE;
	if(Rx_Head == Rx_Tail)
		Rx_Tail = (Rx_Tail + 1) % UART_RX_BUFFER_SIZE;
}

uint8_t ReadBuffer(){
	if(Rx_Tail == Rx_Head){
		Rx_Head = 0;
		Rx_Tail = 0;
		return 0;
	}		
	uint8_t data = Rx_buffer[Rx_Tail];
	Rx_Tail = (Rx_Tail + 1) % UART_RX_BUFFER_SIZE;
	return data;
}






uint8_t Send_sign[] = "send fail\r\n";
uint8_t Success_sign[] = "success\r\n";
uint8_t Error_sign[] = "error\r\n";
uint8_t None_sign[] = "none\r\n";

uint8_t AT_COMMAND(uint8_t *cmd, uint8_t repeat, uint16_t timeout){


	while(repeat > 0){
		memset(BBOX, 0, sizeof(BBOX));
		BBOX_count = 0;
		if(HAL_UART_Transmit(&huart1, cmd, strlen((char *)cmd), 100) == HAL_OK) {
			break;
		}
		repeat--;
	}
	if(repeat == 0){
		//HAL_UART_Transmit(&huart2, Send_sign, strlen((char *)Send_sign), 100);
		//printf("send fail\r\n");
		return false;
	}
	while(timeout > 0){

		if (strstr((char *)BBOX, "OK") != 0){
			//HAL_UART_Transmit(&huart2, Success_sign, strlen((char *)Success_sign), 100);
			//printf("\r\nOK\r\n");
			return true;
		}
		// ERROR -> FAIL
		else if (strstr((char *)BBOX, "FAIL") != 0){
			//HAL_UART_Transmit(&huart2, Error_sign, strlen((char *)Error_sign), 100);
			//printf("\r\nError\r\n");
			return false;
		}

		timeout -= 10;
		HAL_Delay(10);
	}
	//printf("\r\nNone\r\n");


	//HAL_UART_Transmit(&huart2, None_sign, strlen((char *)None_sign), 100);
	//HAL_UART_Transmit(&huart2, BBOX, strlen((char *)BBOX), 100);

	return false;
}

// AT
uint8_t SendAT() {
	uint8_t res = AT_COMMAND((uint8_t *)"AT\r\n", 10, 1000);
	if(res)
		return false;
	return true;
}


// AT+RST
uint8_t EspReset() {
	uint8_t res = AT_COMMAND((uint8_t *)"AT+RST\r\n", 10, 10000);
	if(res)
		return false;
	return true;
}



// AT CWJAP
uint8_t WifiAccess() {
	//uint8_t res = AT_COMMAND((uint8_t *)"AT+CWLAP\r\n" , 10, 1000);
	uint8_t res = AT_COMMAND((uint8_t *)"AT+CWJAP=\"801em1\",\"ssafy1357\"\r\n" , 10, 1000);
//	uint8_t res = AT_COMMAND((uint8_t *)"AT+CWJAP=\"vnqzl\",\"104vnqzl\"" , 10, 1000);
	if(res)
		return false;
	return true;
}


uint8_t ConnectTcp(uint8_t is_MUX, uint8_t target) {
	uint8_t res = 0U;


	// AT_COMMAND((uint8_t *)"AT\r\n", 10, 1000);

	if(is_MUX) { // multi
		if(target == 0) { // rasp
			res = AT_COMMAND((uint8_t *)"AT+CIPSTART=0,\"TCP\",\"192.168.0.41\",12345\r\n", 10, 1000);
		}
		else { // AWS
			res = AT_COMMAND((uint8_t *)"AT+CIPSTART=1,\"TCP\",\"0.0.0.0\",0\r\n", 10, 1000);
		}
	}
	else { // single
//		res = AT_COMMAND((uint8_t *)"AT+CIPSTART=0,\"TCP\",\"192.168.0.41\",12345\r\n", 10, 1000);
		res = AT_COMMAND((uint8_t *)"AT+CIPSTART=\"TCP\",\"192.168.0.41\",12345\r\n", 10, 1000);
	}

	if(res)
		return false;
	return true;
}

// Rasp Send data size & data // target => 0 is rasp // 1 is AWS    // data is real data
uint8_t SendData(uint8_t size, uint8_t is_MUX, uint8_t target, uint8_t * data) {
	uint8_t str[128] = {0, };
	uint8_t res = 0U;
	uint8_t res1 = 0U;

	if(is_MUX == 0) {
		sprintf((char *)str, "AT+CIPSEND=%u\r\n", size);

		res = AT_COMMAND(str, 10, 1000);

		res1 = AT_COMMAND(data, 10, 1000);
	}
	else {
		sprintf((char *)str, "AT+CIPSEND=%u,%u\r\n", target, size);

		res = AT_COMMAND(str, 10, 1000);

		res1 = AT_COMMAND(data, 10, 1000);
	}


	if(res || res1)
		return false;
	return true;
}



// MUX mode
uint8_t SetMux(uint8_t flag) {
	uint8_t res = 0U;

	if(flag) { // MUX
		res = AT_COMMAND((uint8_t *)"AT+MUX=1\r\n", 10, 1000);
	}
	else {
		res = AT_COMMAND((uint8_t *)"AT+MUX=0\r\n", 10, 1000);
	}
	if(res)
		return false;
	return true;
}


uint8_t RaspiTCPSocketAccess() {
	uint8_t res = AT_COMMAND((uint8_t *)"AT+CIPSTART=\"TCP\",\"192.168.0.41\",12345\r\n" , 10, 1000);
//	uint8_t res = AT_COMMAND((uint8_t *)"AT+CIPSTART=0,\"TCP\",\"192.168.0.41\",12345\r\n" , 10, 1000);
	if(res)
		return false;
	return true;
}


uint8_t SendDataToRasp(uint8_t size, uint8_t * data) {
	uint8_t str[128] = {0, };

	sprintf((char *)str, "AT+CIPSEND=%u\r\n", size);
	uint8_t res = AT_COMMAND(str, 10, 1000);
	uint8_t res1 = AT_COMMAND(data, 10, 1000);

	if(res || res1)
		return false;
	return true;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance == USART1) {
		// !!!- DO not printf Here -!!!
		BBOX[BBOX_count++] = buff;
		WriteBuffer(buff);
		HAL_UART_Receive_IT(huart, &buff, 1);
	}

}


void setInterrupt() {
	HAL_UART_Receive_IT(&huart1, &buff, 1);
}

uint8_t EspResponseCheck() {
	while(1) {
		uint8_t data = ReadBuffer();
		if(data != 0){
//#if DEBUG
//			printf("%c", data);
//#endif
			if((char)data == '+'){
				if(strncmp(&Rx_buffer[Rx_Tail], "IPD", 3) == 0){
					uint8_t received_data = Rx_buffer[Rx_Tail+6] - '0';
					Rx_Head = 0;
					Rx_Tail = 0;
					return received_data;
//					uint8_t str[128] = {0, };
//					uint8_t received_len = Rx_buffer[Rx_Tail+4]-'0';
//					for(int i = 0; i < received_len; i++){
//						str[i] = Rx_buffer[Rx_Tail+6 + i];
//					}
//					str[received_len] = '\0';
//					printf("received msg : %s\r\n", str);
//					memset(str,0,sizeof(str));
				}
				else
					continue;
			}
		}
		else
			return 0;
	}
}

void EspSetting() {
	setInterrupt();

	SendAT();
	HAL_Delay(100);

	WifiAccess();
	HAL_Delay(5000);


}
