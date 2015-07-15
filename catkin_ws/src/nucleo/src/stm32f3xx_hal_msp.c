#include "main.h"


void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  static DMA_HandleTypeDef DmaHandle;
  RCC_PeriphCLKInitTypeDef RCC_PeriphCLKInitStruct;

  if (hadc->Instance == ADC1)
  {
    // Enable clock of GPIO associated to the peripheral channels.
    __GPIOA_CLK_ENABLE();

    // Enable clock of ADC peripheral.
    __ADC1_CLK_ENABLE();

    // Enable asynchronous clock source of ADC.
    RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    RCC_PeriphCLKInitStruct.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
    HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);

    // Enable clock of DMA associated to the peripheral.
    __DMA1_CLK_ENABLE();

    // Configure GPIO pin of the selected ADC channel.
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure DMA parameters.
    DmaHandle.Instance = DMA1_Channel1;

    DmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
    DmaHandle.Init.MemInc = DMA_MINC_ENABLE;

    // Transfer from ADC by half-word to match with ADC resolution 10 or 12 bits.
    DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;

    // Transfer to memory by half-word to match with buffer variable type:
    // half-word.
    DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    DmaHandle.Init.Mode = DMA_CIRCULAR;
    DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;

    // Deinitialize and initialize the DMA for new transfer.
    HAL_DMA_DeInit(&DmaHandle);
    HAL_DMA_Init(&DmaHandle);

    /* Associate the initialized DMA handle to the ADC handle */
    __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

    // NVIC configuration for DMA interrupt (transfer completion or error).
    // Priority: high-priority.
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);


    // NVIC configuration for ADC interrupt.
    // Priority: high-priority.
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  }
}


void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    // Reset peripherals.
    __ADC1_FORCE_RESET();
    __ADC1_RELEASE_RESET();

    // De-initialize GPIO pin of the selected ADC channel.
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

    // De-Initialize the DMA associated to the peripheral.
    if(hadc->DMA_Handle != NULL)
    {
      HAL_DMA_DeInit(hadc->DMA_Handle);
    }

    // Disable the NVIC configuration for DMA interrupt.
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

    // Disable the NVIC configuration for ADC interrupt.
    HAL_NVIC_DisableIRQ(ADC1_2_IRQn);
  }
}


void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if(huart->Instance == USART2)
  {
    // Enable peripheral clock.
    __USART2_CLK_ENABLE();

    // USART2 GPIO Configuration.
    // PA2 ------> USART2_TX
    // PA3 ------> USART2_RX
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}


void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
  if(huart->Instance == USART2)
  {
    // Disable peripheral clock.
    __USART2_CLK_DISABLE();

    // USART2 GPIO Configuration.
    // PA2 ------> USART2_TX
    // PA3 ------> USART2_RX
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
  }
}
