#include "watchdog.h"


void WWDG_Init(void)
{
  wwdg.Instance = WWDG;

  // WWDG clock counter = (PCLK1 (32MHz)/4096)/8) = 976 Hz (~1024 us)
  // WWDG Window value = 80 means that the WWDG counter should be refreshed
  // only when the counter is below 80 (and greater than 64/0x40) otherwise a
  // reset will be generated.
  // WWDG Counter value = 127, WWDG timeout = ~1024 us * 64 = 65.57 ms.
  wwdg.Init.Prescaler = WWDG_PRESCALER_8;
  wwdg.Init.Window = 80;
  wwdg.Init.Counter = 127;

  if (HAL_WWDG_Init(&wwdg) != HAL_OK)
  {
    Error_Handler("Could not initialize WWDG");
  }
}


void WWDG_Start(void)
{
  if (HAL_WWDG_Start(&wwdg) != HAL_OK)
  {
    Error_Handler("Could not start WWDG");
  }
}


void WWDG_Stop(void)
{
  if (HAL_WWDG_Stop(&wwdg) != HAL_OK)
  {
    Error_Handler("Could not stop WWDG");
  }
}


void WWDG_Refresh(void)
{
    // Refresh WWDG: update counter value to 127, the refresh window is:
    // between 48.1ms (~1024 * (127-80)) and 65.57 ms (~1024 * 64).
    if (HAL_WWDG_Refresh(&wwdg, 127) != HAL_OK)
    {
      Error_Handler("Could not refresh WWDG");
    }
}
