
#include "stm32f1xx_hal.h"

TIM_HandleTypeDef htim2;

void Error_Handler(void) { } // useful for gdb breakpoint

void TIM2_PWM1_Init(void)
{
  // TIM2 GPIO Configuration
  //   PA0-WKUP  <----> TIM2_CH1
  // So we initiate GPIO PA0 for output PWM
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Now, we initiate TIM2 for PWM on Channel 1
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  { Error_Handler(); }

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  { Error_Handler(); }

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  { Error_Handler(); }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)!=HAL_OK)
  { Error_Handler(); }

  __HAL_RCC_TIM2_CLK_ENABLE();	// start IM2's clock

  if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1)!=HAL_OK) // start PWM on Channel 1
  { Error_Handler(); }
}


