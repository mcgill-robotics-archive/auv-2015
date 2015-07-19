#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include "main.h"
#include "stm32f3xx_hal.h"

static WWDG_HandleTypeDef wwdg;

void WWDG_Init(void);
void WWDG_Start(void);
void WWDG_Stop(void);

#endif
