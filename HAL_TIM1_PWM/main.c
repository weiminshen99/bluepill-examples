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


void GPIO_Init();
HAL_StatusTypeDef TIM1_PWM1_Init();
void SystemClock_Config();

TIM_HandleTypeDef tim1_handle;

// ======================
int main(void) {

  HAL_Init();
  SystemClock_Config();

/*
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
*/

  GPIO_Init();
  TIM1_PWM1_Init();

  if (HAL_InitTick(0) != HAL_ERROR) {
	// means the timers are started successfully 
  }

  while(1)
    {
	HAL_Delay(100);	// is it about 500 ms?
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // PC13 LED
    }
}


// ===================================
// this is needed for HAL_Delay()
void SysTick_Handler(void)	// this SysTick Timer is running at _MHZ
{
  HAL_IncTick();                  	// at Hardware Abstraction Level (HAL), OR
  //extern int32_t uwTick; uwTick++;	// equivelently, do it at the hardware level
}

// System Clock Configuration
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  // Initializes the CPU, AHB and APB busses clocks
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  // Initializes the CPU, AHB and APB busses clocks
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
  // Configure the Systick interrupt time
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
  // Configure the Systick
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  // SysTick_IRQn interrupt configuration
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void GPIO_Init()
{
  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  // PC13 for Output to LED
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // <== OUTPUT_PP below
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // AF_PP pins below
  // PA8 for TIM1's Channle 1 to output PWM
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // PB13 for TIM1's Channel 1 complementary to output ~PWM
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// ======= TIM1 ===========
HAL_StatusTypeDef TIM1_PWM1_Init()
{
  __HAL_RCC_TIM1_CLK_ENABLE();  // start TIM1's clock

  // Initialize TIM1 in a similar way as above
  tim1_handle.Instance = TIM1;
  tim1_handle.Init.Period = 2000;
  tim1_handle.Init.Prescaler = 0; // xN makes it N time slower
  tim1_handle.Init.RepetitionCounter = 0;
  tim1_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim1_handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  tim1_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&tim1_handle) == HAL_ERROR) return HAL_ERROR;
  if (HAL_TIM_PWM_Init (&tim1_handle) == HAL_ERROR) printf("BBBBADD");

  TIM_MasterConfigTypeDef sMasterConfig;
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&tim1_handle, &sMasterConfig);

  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 100; // make it visible
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  HAL_TIM_PWM_ConfigChannel(&tim1_handle, &sConfigOC, TIM_CHANNEL_1);

  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 48; // DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&tim1_handle, &sBreakDeadTimeConfig);

  // Enable all three channels for PWM output
  HAL_TIM_PWM_Start(&tim1_handle, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&tim1_handle, TIM_CHANNEL_1);

  // setup and start TIM1's UPDATE interrupts
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  //__HAL_RCC_TIM1_CLK_ENABLE();
  return HAL_TIM_Base_Start_IT(&tim1_handle);
}

void TIM1_UP_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim1_handle);
  // begin user code
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // toggle PC13/LED
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET); // turn ON PC13/LED
  // end user code
  __HAL_TIM_CLEAR_IT(&tim1_handle, TIM_IT_UPDATE);
}

