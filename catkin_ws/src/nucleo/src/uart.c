#include "uart.h"

UART_HandleTypeDef uart;


/**
 * Initialize USART2.
 */
void UART_Init(void)
{
  uart.Instance = USART2;
  uart.Init.BaudRate = 115200;
  uart.Init.WordLength = UART_WORDLENGTH_8B;
  uart.Init.StopBits = UART_STOPBITS_1;
  uart.Init.Parity = UART_PARITY_NONE;
  uart.Init.Mode = UART_MODE_TX_RX;
  uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart.Init.OverSampling = UART_OVERSAMPLING_16;
  uart.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&uart);
}

/**
 * Write buffer to UART.
 */
void write_buffer(uint8_t* data, uint16_t size) {
  HAL_UART_Transmit(&uart, data, size, 1000);
}

/**
 * Write integer to UART.
 */
void write_int(int data) {
  char buff[10];
  itoa(data, buff, 10);
  HAL_UART_Transmit(&uart, buff, strlen(buff), 1000);
}
