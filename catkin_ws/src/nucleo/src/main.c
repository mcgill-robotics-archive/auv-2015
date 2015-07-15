#include "main.h"

// Maintain status of signals.
__IO uint32_t completion_count[4] = {0, 0, 0, 0};


int main(void)
{
  HAL_Init();

  // Configure the system clock.
  SystemClock_Config();

  // Initialize LED on board.
  BSP_LED_Init(LED2);

  // Initialize UART.
  UART_Init();
  log_debug("Initialized UART");

  // Initialize GPIO.
  log_debug("Initializing GPIO");
  GPIO_Init();

  // Initialize DMA.
  log_debug("Initializing DMA");
  DMA_Init();

  // Configure the ADC peripherals.
  log_debug("Configuring ADCs");
  ADC_Config(&hadc1, ADC1, ADC_CHANNEL_14);
  ADC_Config(&hadc2, ADC2, ADC_CHANNEL_12);
  ADC_Config(&hadc3, ADC3, ADC_CHANNEL_15);
  ADC_Config(&hadc4, ADC4, ADC_CHANNEL_13);

  // Calibrate ADCs.
  log_debug("Calibrating ADCs");
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler("Could not calibrate ADC1");
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler("Could not calibrate ADC2");
  }
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler("Could not calibrate ADC3");
  }
  if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler("Could not calibrate ADC4");
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
  if (HAL_ADC_Start_DMA(&hadc2,
                        (uint32_t *)signals.data_1,
                        BUFFERSIZE) != HAL_OK)
  {
    Error_Handler("Could not start ADC2");
  }
  if (HAL_ADC_Start_DMA(&hadc3,
                        (uint32_t *)signals.data_2,
                        BUFFERSIZE) != HAL_OK)
  {
    Error_Handler("Could not start ADC3");
  }
  if (HAL_ADC_Start_DMA(&hadc4,
                        (uint32_t *)signals.data_3,
                        BUFFERSIZE) != HAL_OK)
  {
    Error_Handler("Could not start ADC4");
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
    | RCC_PERIPHCLK_ADC12
    | RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCOSOURCE_SYSCLK, RCC_MCO_DIV1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}


void DMA_Init(void)
{
  // Enable DMA controller clocks.
  __DMA1_CLK_ENABLE();
  __DMA2_CLK_ENABLE();
}


void GPIO_Init(void)
{
  // Enable GPIO port clocks.
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
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
  char reason[128];

  // Get human readable state.
  char* human_readable_state;
  HAL_ADC_StateTypeDef state = HAL_ADC_GetState(hadc);
  switch (state) {
    case HAL_ADC_STATE_RESET:
      human_readable_state = "RESET";
      break;
    case HAL_ADC_STATE_READY:
      human_readable_state = "READY";
      break;
    case HAL_ADC_STATE_BUSY:
      human_readable_state = "BUSY";
      break;
    case HAL_ADC_STATE_BUSY_REG:
      human_readable_state = "BUSY_REG";
      break;
    case HAL_ADC_STATE_BUSY_INJ:
      human_readable_state = "BUSY_INJ";
      break;
    case HAL_ADC_STATE_BUSY_INJ_REG:
      human_readable_state = "BUSY_INJ_REG";
      break;
    case HAL_ADC_STATE_TIMEOUT:
      human_readable_state = "TIMEOUT";
      break;
    case HAL_ADC_STATE_ERROR:
      human_readable_state = "ERROR";
      break;
    case HAL_ADC_STATE_EOC:
      human_readable_state = "EOC";
      break;
    case HAL_ADC_STATE_EOC_REG:
      human_readable_state = "EOC_REG";
      break;
    case HAL_ADC_STATE_EOC_INJ:
      human_readable_state = "EOC_INJ";
      break;
    case HAL_ADC_STATE_EOC_INJ_REG:
      human_readable_state = "EOC_INJ_REG";
      break;
    case HAL_ADC_STATE_AWD:
      human_readable_state = "AWD";
      break;
    case HAL_ADC_STATE_AWD2:
      human_readable_state = "AWD2";
      break;
    case HAL_ADC_STATE_AWD3:
      human_readable_state = "AWD3";
      break;
    default:
      sprintf(human_readable_state, "%u", state);
      break;
  }

  // Get human readable error.
  char* human_readable_error;
  uint32_t error = HAL_ADC_GetError(hadc);
  switch (error) {
    case HAL_ADC_ERROR_NONE:
      human_readable_error = "NONE";
      break;
    case HAL_ADC_ERROR_INTERNAL:
      human_readable_error = "INTERNAL";
      break;
    case HAL_ADC_ERROR_OVR:
      human_readable_error = "OVR";
      break;
    case HAL_ADC_ERROR_DMA:
      human_readable_error = "DMA";
      break;
    case HAL_ADC_ERROR_JQOVF:
      human_readable_error = "JQOVF";
      break;
    default:
      sprintf(human_readable_error, "%u", error);
      break;
  }

  // Handle error.
  sprintf(reason, "ADC ERROR CALLBACK: (state: %s, error: %s)",
          human_readable_state, human_readable_error);
  Error_Handler(reason);
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
