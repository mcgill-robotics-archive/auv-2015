#include "stm32f3xx_hal.h"

UART_HandleTypeDef uart;

void UART_Init(void);
void write_int(int);
void write_buffer(uint8_t*, uint16_t);
