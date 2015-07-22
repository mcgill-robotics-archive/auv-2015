#include "adc.h"

// Compute half the buffersize to speed up ping search.
static const uint32_t HALF_BUFFERSIZE = BUFFERSIZE / 2;

// Compute double the buffersize to speed up sending the buffer.
static const uint32_t DOUBLE_BUFFERSIZE = 2 * BUFFERSIZE;

// Determine measurement size depending on resolution.
#ifdef TWELVE_BIT_MODE
static const uint8_t MEASUREMENT_SIZE = 2;
#else
static const uint8_t MEASUREMENT_SIZE = 1;
#endif


void ADC_Config(ADC_HandleTypeDef* hadc, ADC_TypeDef* adc)
{
  hadc->Instance = adc;

  hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC;
#ifdef TWELVE_BIT_MODE
  hadc->Init.Resolution = ADC_RESOLUTION12b;
#else
  hadc->Init.Resolution = ADC_RESOLUTION8b;
#endif
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
    Error_Handler("Could not initialize ADC");
  }
}


void Add_ADC_Channel(ADC_HandleTypeDef* hadc, uint32_t channel, uint32_t rank)
{
  ADC_ChannelConfTypeDef sConfig;

  sConfig.Channel = channel;
  sConfig.Rank = rank;

  // Sampling time in ADC clock cycles.
  // This value is added to a constant ADC clock cycle count dependent on the
  // ADC resolution:
  //  12 bit: 12.5 ADC clock cycles.
  //  8 bit: 8.5 ADC clock cycles.
  #ifdef TWELVE_BIT_MODE
    // 12.5 + 61.5 = 74 ADC clock cycles --> 72 MHz / 74 = 972 972.97297 Hz.
    // Experimentally, at 72MHz, this is on average approximately 971 959 Hz.
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  #else
    // 8.5 + 61.5 = 70 ADC clock cycles --> 72 MHz / 70 = 1 028 571.4286 Hz.
    // Experimentally, at 72MHz, this is on average approximately 1 027 527 Hz.
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  #endif

  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
  {
    char* error;
    uint8_t instance = Get_ADC_Instance(hadc);
    sprintf(error, "Could not configure ADC%u channel", instance);
    Error_Handler(error);
  }
}


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


void Start_ADC(ADC_HandleTypeDef* hadc, uint32_t* buff)
{
  // Try to start.
  if (HAL_ADC_Start_DMA(hadc, buff, BUFFERSIZE) != HAL_OK)
  {
    char* error;
    uint8_t instance = Get_ADC_Instance(hadc);
    sprintf(error, "Could not start ADC%u", instance);
    Error_Handler(error);
  }

  // Increment counter.
  active_adcs++;
}


void Stop_ADC(ADC_HandleTypeDef* hadc)
{
  // Try to stop.
  if (HAL_ADC_Stop_DMA(hadc) != HAL_OK)
  {
    char* error;
    uint8_t instance = Get_ADC_Instance(hadc);
    sprintf(error, "Could not start ADC%u", instance);
    Error_Handler(error);
  }

  // Decrement counter.
  active_adcs--;
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  // Conversion is complete.

  // Verfify if ping had been detected in previous half callback.
  if (found_ping && !buffer_inverted)
  {
    // If so, stop this ADC.
    Stop_ADC(hadc);
  }

  // Write data if all ADCs have been stopped and restart ADCs.
  if (active_adcs == 0)
  {
    write_buffer(Get_Quadrant_Header(&hadc1), 9);
    for (int i = 0; i < DOUBLE_BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_1 + i, MEASUREMENT_SIZE);
    }

    write_buffer(Get_Quadrant_Header(&hadc2), 9);
    for (int i = 0; i < DOUBLE_BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_2 + i, MEASUREMENT_SIZE);
    }

    write_buffer(Get_Quadrant_Header(&hadc3), 9);
    for (int i = 0; i < DOUBLE_BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_3 + i, MEASUREMENT_SIZE);
    }

    write_buffer(Get_Quadrant_Header(&hadc4), 9);
    for (int i = 0; i < DOUBLE_BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_4 + i, MEASUREMENT_SIZE);
    }

    // Restart ADCs.
    Start_ADC(&hadc1, (uint32_t*) data_1);
    Start_ADC(&hadc2, (uint32_t*) data_2);
    Start_ADC(&hadc3, (uint32_t*) data_3);
    Start_ADC(&hadc4, (uint32_t*) data_4);

    // Reset flag.
    found_ping = 0;
    buffer_inverted = 0;
  }
  // Otherwise determine if there is a ping in the second half.
  else
  {
    uint8_t instance = Get_ADC_Instance(hadc);
    uint32_t* buffer_pointer;
    switch (instance) {
      case 1:
        buffer_pointer = (uint32_t*) data_1 + HALF_BUFFERSIZE;
        break;
      case 2:
        buffer_pointer = (uint32_t*) data_2 + HALF_BUFFERSIZE;
        break;
      case 3:
        buffer_pointer = (uint32_t*) data_3 + HALF_BUFFERSIZE;
        break;
      case 4:
        buffer_pointer = (uint32_t*) data_4 + HALF_BUFFERSIZE;
        break;
      default:
        // Dealt with in Get_ADC_Instance().
        return;
    }
    found_ping = has_ping(buffer_pointer, HALF_BUFFERSIZE, energy_threshold);
    if (found_ping)
    {
      buffer_inverted = 1;
    }
  }
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  // ADC conversion half-complete.

  // Verfify if ping had been detected in previous complete callback.
  if (found_ping && buffer_inverted)
  {
    // If so, stop this ADC.
    Stop_ADC(hadc);
  }

  // Write data if all ADCs have been stopped and restart ADCs.
  if (active_adcs == 0)
  {
    write_buffer(Get_Quadrant_Header(&hadc1), 9);
    for (int i = BUFFERSIZE; i < DOUBLE_BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_1 + i, MEASUREMENT_SIZE);
    }
    for (int i = 0; i < BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_1 + i, MEASUREMENT_SIZE);
    }

    write_buffer(Get_Quadrant_Header(&hadc2), 9);
    for (int i = BUFFERSIZE; i < DOUBLE_BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_2 + i, MEASUREMENT_SIZE);
    }
    for (int i = 0; i < BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_2 + i, MEASUREMENT_SIZE);
    }

    write_buffer(Get_Quadrant_Header(&hadc3), 9);
    for (int i = BUFFERSIZE; i < DOUBLE_BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_3 + i, MEASUREMENT_SIZE);
    }
    for (int i = 0; i < BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_3 + i, MEASUREMENT_SIZE);
    }

    write_buffer(Get_Quadrant_Header(&hadc4), 9);
    for (int i = BUFFERSIZE; i < DOUBLE_BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_4 + i, MEASUREMENT_SIZE);
    }
    for (int i = 0; i < BUFFERSIZE; i += 2) {
      write_buffer((uint8_t*) data_4 + i, MEASUREMENT_SIZE);
    }

    // Restart ADCs.
    Start_ADC(&hadc1, (uint32_t*) data_1);
    Start_ADC(&hadc2, (uint32_t*) data_2);
    Start_ADC(&hadc3, (uint32_t*) data_3);
    Start_ADC(&hadc4, (uint32_t*) data_4);

    // Reset flags.
    found_ping = 0;
    buffer_inverted = 0;

    return;
  }

  // Determine if ping is available in first half of ping.
  uint8_t instance = Get_ADC_Instance(hadc);
  uint32_t* buffer_pointer;
  switch (instance) {
    case 1:
      buffer_pointer = (uint32_t*) data_1;
      break;
    case 2:
      buffer_pointer = (uint32_t*) data_2;
      break;
    case 3:
      buffer_pointer = (uint32_t*) data_3;
      break;
    case 4:
      buffer_pointer = (uint32_t*) data_4;
      break;
    default:
      // Dealt with in Get_ADC_Instance().
      return;
  }
  found_ping = has_ping(buffer_pointer, HALF_BUFFERSIZE, energy_threshold);
  if (found_ping)
  {
    buffer_inverted = 0;
  }
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


char* Get_Quadrant_Header(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == QUADRANT_I) {
    return "[DATA 1]";
  }
  else if (hadc->Instance == QUADRANT_II) {
    return "[DATA 2]";
  }
  else if (hadc->Instance == QUADRANT_III) {
    return "[DATA 3]";
  }
  else if (hadc->Instance == QUADRANT_IV) {
    return "[DATA 4]";
  }
  else {
    // Oops...
    Error_Handler("Could not determine quadrant");
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
