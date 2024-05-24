/*
 * IO_Init.h
 *
 *  Created on: Jan 25, 2024
 *      Author: Kim YR
 */

#ifndef USERLIB_INC_IO_INIT_H_
#define USERLIB_INC_IO_INIT_H_

#include <stdio.h>
#include "stm32f1xx_hal.h"

void Init_printf_PFP(void);
void Init_printf_scnaf_PFP(void);
void Init_printf_scnaf_main(void);

#endif /* USERLIB_INC_IO_INIT_H_ */
