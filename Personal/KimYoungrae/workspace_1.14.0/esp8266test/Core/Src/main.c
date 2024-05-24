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
#include <stdio.h>
#include <string.h>
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
void sendATCommand(const char *command);
HAL_StatusTypeDef receiveResponse(char *response, uint16_t timeout);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  uint8_t AT[20] = {0, };
  AT[0] = 'A';
  AT[1] = 'T';

  uint8_t rev[100] = {0, };
  // uint8_t s[256] = {0, };

  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);

  // AP?��?��

  printf("send AT\r\n");

  //strlen((char *)buffer)
  //HAL_UART_Transmit(&huart3, AT, strlen((char *)AT), 1000);
  uint8_t Tret = HAL_UART_Transmit(&huart3, AT, strlen((char *)AT), HAL_MAX_DELAY);
  //HAL_StatusTypeDef Tret = HAL_UART_Transmit_IT(&huart3, AT, strlen((char *)AT));
  //uint8_t Tret = HAL_UART_Transmit_IT(&huart3, AT, strlen((char *)AT));

  if(Tret == 0x00) {
	  printf("Trans OK \r\n");
  }
  else {
	  printf("Trans Error \r\n");
  }

  //printf("%s", WIFI_ACCESS);
  //HAL_Delay(10);


  rev[0] = 0;
  //HAL_UART_Receive(&huart3, rev, 100, 1000);
  uint8_t ret = HAL_UART_Receive(&huart3, rev, 5, HAL_MAX_DELAY);
  //HAL_StatusTypeDef ret = HAL_UART_Receive_IT(&huart3, rev, 5);
  //uint8_t ret = HAL_UART_Receive_IT(&huart3, rev, 5);
  HAL_Delay(100);

  //printf("RE %lu\r\n", ret);
  if(ret == 0x00) {
  	printf("REC OK \r\n");
  }
  else {
  	printf("REC Error \r\n");
  }
  printf("AP : %c \r\n", rev[0]);
  memset(rev, 0, 100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  char response[50];

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //sendATCommand("AT");

	  // ?��?��?�� 받습?��?��.
	  //if (receiveResponse(response, 1000) == HAL_OK)
	  //{
	    // "OK" ?��?��?�� STM32?���??? ?��?��?��?��?��.
	  //  HAL_UART_Transmit(&huart3, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
	  //}
	  printf("... \r\n");

	  HAL_Delay(1000);

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
}

/* USER CODE BEGIN 4 */
void sendATCommand(const char *command)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);
  //HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef receiveResponse(char *response, uint16_t timeout)
{
  uint8_t rxBuffer[50];
  uint32_t startTime = HAL_GetTick();

  memset(response, 0, sizeof(response));

  while (HAL_GetTick() - startTime < timeout)
  {
    if (HAL_UART_Receive(&huart3, rxBuffer, sizeof(rxBuffer), 100) == HAL_OK)
    {
      strcat(response, (char *)rxBuffer);

      // "OK" ?��?��?�� ?��?��?��?��?���??? ?��?��?��?��?��.
      if (strstr(response, "OK") != NULL)
      {
    	  printf("OK found!\r\n");
        return HAL_OK;
      }
    }
  }
  printf("Cant access\r\n");
  return HAL_TIMEOUT;
}
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
