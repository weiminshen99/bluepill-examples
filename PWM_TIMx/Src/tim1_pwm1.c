// **************************************************************
// Copyright (c) 2022 AARI Corporation. All rights reserved
// Author Wei-Min Shen, 3-26-2022, 11-9-2022
//
// A demo for two timers to colaborate with each other:
// 	Timer1's interrupt trun on  PC13/LED, and
// 	Timer2's interrupt turn off PC13/LED
//	To see these fast effects, gdb break on HAL_GPIO_TogglePin
// ***************************************************************

#include "stm32f1xx_hal.h"
#include "main.h"
#include "tim1_pwm1.h"

TIM_HandleTypeDef tim1_handle;

void TIM1_PWM1_Init()
{
  // Initialize TIM1 in a similar way as above
  tim1_handle.Instance = TIM1;
  tim1_handle.Init.Prescaler = 0; // xN makes it N time slower
  tim1_handle.Init.CounterMode = TIM_COUNTERMODE_UP; // TIM_COUNTERMODE_CENTERALIGNED1;
  tim1_handle.Init.Period = 65535;
  tim1_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim1_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  //tim1_handle.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&tim1_handle) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&tim1_handle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&tim1_handle) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // TIM_TRGO_ENABLE, RESET ?
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&tim1_handle,&sMasterConfig)!=HAL_OK)
  {
    Error_Handler();
  }

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode   = TIM_OCFAST_ENABLE; 	// TIM_OCFAST_DISABLE ?
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&tim1_handle,&sConfigOC,TIM_CHANNEL_1)!=HAL_OK)
  {
    Error_Handler();
  }

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 48; // DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&tim1_handle,&sBreakDeadTimeConfig)!=HAL_OK)
  {
    Error_Handler();
  }

  // Enable all three channels for PWM output
  HAL_TIM_PWM_Start(&tim1_handle, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&tim1_handle, TIM_CHANNEL_1);

  //TIM1->BDTR &= ~TIM_BDTR_MOE;
  __HAL_TIM_ENABLE(&tim1_handle);

  //__HAL_RCC_TIM1_CLK_ENABLE();  // start TIM1's clock, make it is in the main loop
}

void TIM1_PWM1_GPIO_Init()
{
  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // AF_PP pins below
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // PA8 for TIM1's Channle 1 to output PWM
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // PB13 for TIM1's Channel 1N to output ~PWM
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
