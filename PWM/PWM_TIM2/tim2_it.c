
#include "stm32f1xx_hal.h"

TIM_HandleTypeDef tim2_handle;

void TIM2_IT_Init()
{
  // Initialize TIM2 Base
  tim2_handle.Instance = TIM2;
  tim2_handle.Init.Period = 72000000;
  tim2_handle.Init.Prescaler = 1;	// xN will slow N times
  tim2_handle.Init.ClockDivision = 0U;
  tim2_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim2_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&tim2_handle);

  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0); // set TIM2 IRQ priority
  HAL_NVIC_EnableIRQ(TIM2_IRQn);	// enable TIM2 global interrupt
  __HAL_RCC_TIM2_CLK_ENABLE();		// enable TIM2 clock
  HAL_TIM_Base_Start_IT(&tim2_handle); // start TIM2 interrupt
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim2_handle);

  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	// toggle LED at PC13
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);	// turn OFF LED at PC13

  __HAL_TIM_CLEAR_IT(&tim2_handle, TIM_IT_UPDATE); // clear TIM interrupt
}

