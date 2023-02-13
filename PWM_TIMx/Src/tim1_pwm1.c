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

// This function init TIM1 and its I/O Pins

#include "defines.h"

TIM_HandleTypeDef htim_right;

void TIM1_and_Pins_Init(void)
{
  htim_right.Instance               = TIM1;
  htim_right.Init.Period            = 64000000 / 2 / 16000;
  htim_right.Init.Prescaler         = 0;
  htim_right.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
  htim_right.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_right.Init.RepetitionCounter = 0;
  htim_right.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim_right) == HAL_ERROR) printf("BBBBADD");

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim_right, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim_right);

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE; // TIM_TRGO_DISABLE
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_right, &sMasterConfig);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_3);

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 48; // DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_right, &sBreakDeadTimeConfig);

  // Enable all three channels for PWM output
  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_3);
  // Enable all three complemenary channels for PWM output
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_3);

  __HAL_TIM_ENABLE(&htim_right);
//  __HAL_RCC_TIM1_CLK_ENABLE();	// already did in main

  // Now we init GPIO Pins for TIM1
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // PA8
  GPIO_InitStruct.Pin = MOTOR_TIM_UH_PIN;
  HAL_GPIO_Init(MOTOR_TIM_UH_PORT, &GPIO_InitStruct);
  // PB13
  GPIO_InitStruct.Pin = MOTOR_TIM_UL_PIN;
  HAL_GPIO_Init(MOTOR_TIM_UL_PORT, &GPIO_InitStruct);
  // PA9
  GPIO_InitStruct.Pin = MOTOR_TIM_VH_PIN;
  HAL_GPIO_Init(MOTOR_TIM_VH_PORT, &GPIO_InitStruct);
  // PB14
  GPIO_InitStruct.Pin = MOTOR_TIM_VL_PIN;
  HAL_GPIO_Init(MOTOR_TIM_VL_PORT, &GPIO_InitStruct);
  // PA10
  GPIO_InitStruct.Pin = MOTOR_TIM_WH_PIN;
  HAL_GPIO_Init(MOTOR_TIM_WH_PORT, &GPIO_InitStruct);
  // PB15
  GPIO_InitStruct.Pin = MOTOR_TIM_WL_PIN;
  HAL_GPIO_Init(MOTOR_TIM_WL_PORT, &GPIO_InitStruct);
}





void TIM1_PWM1_GPIO_Init()
{
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

void TIM1_PWM1_Init()
{
  // Initialize TIM1 in a similar way as above
  tim1_handle.Instance = TIM1;
  tim1_handle.Init.Period = 65535;
  tim1_handle.Init.Prescaler = 0; // xN makes it N time slower
  tim1_handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1; // TIM_COUNTERMODE_UP; //
  tim1_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim1_handle.Init.RepetitionCounter = 0;
  tim1_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE; // TIM_TRGO_ENABLE, RESET ?
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // TIM_TRGO_ENABLE, RESET ?
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
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE; // TIM_OCFAST_ENABLE; // TIM_OCFAST_DISABLE ?
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
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE; // DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&tim1_handle,&sBreakDeadTimeConfig)!=HAL_OK)
  {
    Error_Handler();
  }

  // Enable all three channels for PWM output
  HAL_TIM_PWM_Start(&tim1_handle, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&tim1_handle, TIM_CHANNEL_1);

  //TIM1->BDTR &= ~TIM_BDTR_MOE;
  __HAL_TIM_ENABLE(&tim1_handle);
  __HAL_RCC_TIM1_CLK_ENABLE();  // start TIM1's clock, already did it in the main loop
}

