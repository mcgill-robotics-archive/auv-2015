#ifndef __ADC_H
#define __ADC_H

#include "main.h"
#include <string.h>
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

#define BUFFERSIZE ((uint32_t) 4096)

__IO uint16_t data_0[BUFFERSIZE];
__IO uint16_t data_1[BUFFERSIZE];
__IO uint16_t data_2[BUFFERSIZE];
__IO uint16_t data_3[BUFFERSIZE];

void ADC_Config(ADC_HandleTypeDef*, ADC_TypeDef*, uint32_t);
void Calibrate_ADC(ADC_HandleTypeDef*);
uint8_t Get_ADC_Instance(ADC_HandleTypeDef*);
void Start_ADC(ADC_HandleTypeDef*, uint32_t*);
void Stop_ADC(ADC_HandleTypeDef*);
char* Get_Human_Readable_State(ADC_HandleTypeDef*);
char* Get_Human_Readable_Error(ADC_HandleTypeDef*);

#endif
