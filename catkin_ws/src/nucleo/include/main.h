#ifndef __MAIN_H
#define __MAIN_H

#include "uart.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_nucleo.h"

// ADC handler declarations.
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;

// ADC DMA handler declarations.
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;
DMA_HandleTypeDef hdma_adc4;

#define BUFFERSIZE ((uint32_t) 1024)

typedef struct {
  __IO uint16_t data_0[BUFFERSIZE];
  __IO uint16_t data_1[BUFFERSIZE];
  __IO uint16_t data_2[BUFFERSIZE];
  __IO uint16_t data_3[BUFFERSIZE];
} SignalData;

void DMA_Init(void);
void GPIO_Init(void);
void Error_Handler(char*);
void SystemClock_Config(void);
void ADC_Config(ADC_HandleTypeDef*, ADC_TypeDef*, uint32_t);

#endif
