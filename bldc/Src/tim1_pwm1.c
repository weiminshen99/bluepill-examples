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
#include "defines.h"

TIM_HandleTypeDef htim_right;

void TIM1_and_Pins_Init(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();

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
  //if (HAL_TIM_PWM_Init(&htim_right) == HAL_ERROR) printf("BBBBADD");

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE; // TIM_TRGO_DISABLE
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_right, &sMasterConfig);

  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH; // TIM_OCNPOLARITY_LOW;
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
