// ---------------------------------
#include "global_define.h"
#include "stm32f1xx_hal.h"
#define UART_RX_BUFFER_SIZE 1024
// ---------------------------------

void setInterrupt();
uint8_t SendAT();
uint8_t EspReset();
uint8_t WifiAccess();
uint8_t ReadBuffer();
