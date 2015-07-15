#include "adc.h"

// Maintain status of signals.
__IO uint32_t completion_count[4] = {0, 0, 0, 0};


void Calibrate_ADC(ADC_HandleTypeDef* hadc)
{
  if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    char* error;
    uint8_t instance = Get_ADC_Instance(hadc);
    sprintf(error, "Could not calibrate ADC%u", instance);
    Error_Handler(error);
  }
}


uint8_t Get_ADC_Instance(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    return 1;
  }
  else if (hadc->Instance == ADC2) {
    return 2;
  }
  else if (hadc->Instance == ADC3) {
    return 3;
  }
  else if (hadc->Instance == ADC4) {
    return 4;
  }
  else {
    // Oops...
    Error_Handler("Could not determine ADC instance");
  }
}


char* Get_Human_Readable_State(ADC_HandleTypeDef* hadc)
{
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
  return human_readable_state;
}


char* Get_Human_Readable_Error(ADC_HandleTypeDef* hadc)
{
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
  return human_readable_error;
}


void Start_ADC(ADC_HandleTypeDef* hadc, uint32_t* buff)
{
  if (HAL_ADC_Start_DMA(hadc, buff, BUFFERSIZE) != HAL_OK)
  {
    char* error;
    uint8_t instance = Get_ADC_Instance(hadc);
    sprintf(error, "Could not start ADC%u", instance);
    Error_Handler(error);
  }
}


void Stop_ADC(ADC_HandleTypeDef* hadc)
{
  if (HAL_ADC_Stop_DMA(hadc) != HAL_OK)
  {
    char* error;
    uint8_t instance = Get_ADC_Instance(hadc);
    sprintf(error, "Could not start ADC%u", instance);
    Error_Handler(error);
  }
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
  uint8_t instance = Get_ADC_Instance(hadc) - 1;

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
  char* human_readable_state = Get_Human_Readable_State(hadc);

  // Get human readable error.
  char* human_readable_error = Get_Human_Readable_Error(hadc);

  // Handle error.
  sprintf(reason, "ADC ERROR CALLBACK: (state: %s, error: %s)",
          human_readable_state, human_readable_error);
  Error_Handler(reason);
}
