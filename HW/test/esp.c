/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int fd, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (unsigned char*)ptr, len, HAL_MAX_DELAY);
	return len;
}

int _read(int file, char *ptr, int len)
{
	HAL_UART_Receive(&huart2, (unsigned char*)ptr, len, HAL_MAX_DELAY);
	return len;
}

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (unsigned char*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

int __io_getchar(void)
{
	uint8_t ch = 0;
	HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */






#define UART_RX_BUFFER_SIZE 1024
#define true 1
#define false 0



uint8_t BBOX[1024] = {0, };
uint8_t buff;
volatile uint8_t BBOX_count = 0;



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
	if(Rx_Tail == Rx_Head)
		return 0;
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
		return false;
	}
	while(timeout > 0){

		if (strstr((char *)BBOX, "OK") != 0){
			return true;
		}
		// ERROR -> FAIL
		else if (strstr((char *)BBOX, "FAIL") != 0){
			return false;
		}

		timeout -= 10;
		HAL_Delay(10);
	}
	return false;
}






// -------------------------- AT command Funtion START ----------------------------------------


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
	uint8_t res = AT_COMMAND((uint8_t *)"AT+CWJAP=\"801em1\",\"ssafy1357\"\r\n" , 10, 1000);
	if(res)
		return false;
	return true;
}


uint8_t ConnectTcp(uint8_t is_MUX, uint8_t target) {
	uint8_t res = 0U;



	if(is_MUX) { // multi
		if(target == 0) { // rasp
			res = AT_COMMAND((uint8_t *)"AT+CIPSTART=0,\"TCP\",\"192.168.0.41\",12345\r\n", 10, 1000);
		}
		else { // AWS
			res = AT_COMMAND((uint8_t *)"AT+CIPSTART=1,\"TCP\",\"0.0.0.0\",0\r\n", 10, 1000);
		}
	}
	else { // single
		res = AT_COMMAND((uint8_t *)"AT+CIPSTART=0,\"TCP\",\"192.168.0.41\",12345\r\n", 10, 1000);
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
	if(res)
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  // Wait for Interrupt
  HAL_UART_Receive_IT(&huart1, &buff, 1);




  WifiAccess();

  HAL_Delay(5000);

  RaspiTCPSocketAccess();

  HAL_Delay(100);

  // SendData(uint8_t size, uint8_t is_MUX, uint8_t target, uint8_t * data)

  SendData(strlen((char *)"ABCDEFG\r\n"), 0, 0, (uint8_t *)"ABCDEFG\r\n");


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // change [1] ==> HAL_UART_Transmit -> HAL_UART_Transmit_IT



	  uint8_t data = ReadBuffer();
	  if(data != 0)
		  printf("%c", data);


	  HAL_Delay(100);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}z

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
