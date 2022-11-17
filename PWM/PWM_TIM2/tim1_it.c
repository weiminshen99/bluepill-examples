
#include "stm32f1xx_hal.h"

TIM_HandleTypeDef tim1_handle;

HAL_StatusTypeDef TIM1_IT_Init()
{
  // Initialize TIM1 in a similar way as above
  tim1_handle.Instance = TIM1;
  tim1_handle.Init.Period = 72000000;
  tim1_handle.Init.Prescaler = 1;	// xN makes it N time slower
  tim1_handle.Init.ClockDivision = 0U;
  tim1_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim1_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&tim1_handle)==HAL_ERROR)
     return HAL_ERROR;

  // setup and start TIM1's UPDATE interrupts
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  __HAL_RCC_TIM1_CLK_ENABLE();
  return HAL_TIM_Base_Start_IT(&tim1_handle);
}

void TIM1_UP_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim1_handle);
  // begin user code
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // toggle PC13/LED
  // end user code
  __HAL_TIM_CLEAR_IT(&tim1_handle, TIM_IT_UPDATE);
}
