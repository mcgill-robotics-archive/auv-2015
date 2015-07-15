#include "uart.h"

/**
 * Initializes USART2.
 */
void UART_Init(void)
{
  uart.Instance = USART2;
  uart.Init.BaudRate = 460800;
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
 * Writes buffer to UART.
 */
void write_buffer(uint8_t* data, uint16_t size) {
  HAL_UART_Transmit(&uart, data, size, 1000);
}


/**
 * Writes string to UART.
 */
void write_string(char* data) {
  HAL_UART_Transmit(&uart, data, strlen(data), 1000);
}


/**
 * Writes integer to UART.
 */
void write_int(int data) {
  char buff[10];
  sprintf(buff, "%d", data);
  HAL_UART_Transmit(&uart, buff, sizeof(buff), 1000);
}


/**
 * Writes debug log to UART.
 */
void log_debug(char* data) {
  #ifdef DEBUG
  HAL_UART_Transmit(&uart, "[DEBUG] ", 8, 1000);
  HAL_UART_Transmit(&uart, data, strlen(data), 1000);
  HAL_UART_Transmit(&uart, "\r\n", 2, 1000);
  #endif
}


/**
 * Writes fatal log to UART.
 */
void log_fatal(char* data) {
  HAL_UART_Transmit(&uart, "[FATAL] ", 8, 1000);
  HAL_UART_Transmit(&uart, data, strlen(data), 1000);
  HAL_UART_Transmit(&uart, "\r\n", 2, 1000);
}
