#ifndef __PEAK_H
#define __PEAK_H

#include "adc.h"
#include <stdlib.h>
#include "stm32f3xx_hal.h"

// 2 seconds between each ping.
#define PING_PERIOD (uint32_t) 2000

// Max voltage as seen per the ADC.
#ifdef TWELVE_BIT_MODE
#define PEAK_TO_PEAK (uint16_t) 4095
#else
#define PEAK_TO_PEAK (uint16_t) 255
#endif

// DC offset as seen per the ADC.
#define DC_OFFSET PEAK_TO_PEAK / 2

uint8_t has_ping(uint32_t*, uint32_t, uint8_t);

#endif
