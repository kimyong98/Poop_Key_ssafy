/*
 * IO_Init.c
 *
 *  Created on: Jan 25, 2024
 *      Author: Kim YR
 */


#include "IO_Init.h"


extern UART_HandleTypeDef huart2;


// Overwrite
// Only Use printf()
void Init_printf_PFP(void) {
	#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#endif

	PUTCHAR_PROTOTYPE
	{
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
		return ch;
	}
}



// Overwrite
// Use printf() and scanf()
void Init_printf_scnaf_PFP(void) {
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
}

void Init_printf_scnaf_main(void) {
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
}
