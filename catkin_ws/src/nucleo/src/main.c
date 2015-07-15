#include "main.h"

// Maintain status of signals.
__IO uint32_t completion_count[4] = {0, 0, 0, 0};

int main(void)
{
  HAL_Init();

  // Configure the system clock to 64 MHz.
  SystemClock_Config();

  // Initialize LED on board.
  BSP_LED_Init(LED2);

  // Initialize UART.
  UART_Init();
  log_debug("Initialized UART");

  // Configure the ADC peripherals.
  log_debug("Configuring ADCs");
  ADC_Config(&hadc1, ADC1, ADC_CHANNEL_TEMPSENSOR);

  // Calibrate ADCs.
  log_debug("Calibrating ADCs");
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler("Could not calibrate ADC1");
  }

  // Start ADC conversion by DMA.
  SignalData signals;
  log_debug("Starting ADCs");
  if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t *)signals.data_0,
                        BUFFERSIZE) != HAL_OK)
  {
    Error_Handler("Could not start ADC1");
  }

  while (1)
  {
    write_buffer("alive\r\n", 7);
    HAL_Delay(1000);
  }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}


void ADC_Config(ADC_HandleTypeDef* hadc, ADC_TypeDef* adc, uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig;

  hadc->Instance = adc;

  hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc->Init.Resolution = ADC_RESOLUTION12b;
  hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc->Init.ScanConvMode = ENABLE;
  hadc->Init.EOCSelection = EOC_SINGLE_CONV;
  hadc->Init.LowPowerAutoWait = DISABLE;
  hadc->Init.ContinuousConvMode = ENABLE;
  hadc->Init.NbrOfConversion = 1;
  hadc->Init.DiscontinuousConvMode = DISABLE;
  hadc->Init.NbrOfDiscConversion = 0;
  hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc->Init.DMAContinuousRequests = ENABLE;
  hadc->Init.Overrun = OVR_DATA_OVERWRITTEN;

  if (HAL_ADC_Init(hadc) != HAL_OK)
  {
    Error_Handler("ADC could not be intialized");
  }

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    Error_Handler("Channel could not be configured");
  }
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  // Conversion is complete.

  // Determine which ADC is complete.
  uint8_t instance = 0;
  if (hadc->Instance == ADC1) {
    instance = 0;
  }
  else if (hadc->Instance == ADC2) {
    instance = 1;
  }
  else if (hadc->Instance == ADC3) {
    instance = 2;
  }
  else if (hadc->Instance == ADC4) {
    instance = 3;
  }
  else {
    // Oops...
    write_buffer("Unknown instance :(\r\n", 21);
    return;
  }

  // Verfify if complete.
  completion_count[instance]++;
  if (completion_count[instance] > 1000) {
    completion_count[instance] = 0;
    write_buffer("DONE ", 5);
    write_int(instance);
    write_buffer("\n", 1);
  }
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  // ADC conversion half-complete.
}


void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  Error_Handler("ADC Error Callback");
}


void Error_Handler(char* reason)
{
  while(1)
  {
    BSP_LED_Toggle(LED2);
    log_fatal(reason);
    HAL_Delay(1000);
  }
}
