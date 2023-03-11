/**
  ******************************************************************************
  * @file    stm32f1xx_hal_timebase_tim_template.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    12-May-2017
  * @brief   HAL time base based on the hardware TIM Template.
  *
  *          This file overrides the native HAL time base functions (defined as weak)
  *          the TIM time base:
  *           + Intializes the TIM peripheral generate a Period elapsed Event each 1ms
  *           + HAL_IncTick is called inside HAL_TIM_PeriodElapsedCallback ie each 1ms
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */

/** @addtogroup HAL_TimeBase_TIM
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef        Tim2Handle;

/* Private function prototypes -----------------------------------------------*/

void TIM2_IRQHandler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  This function configures the TIM2 as a time base source.
  *         The time source is configured to have 1ms time base with a dedicated
  *         Tick interrupt priority.
  * @note   This function is called automatically at the beginning of program after
  *         reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
  * @param  TickPriority: Tick interrupt priority.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority)
{
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock, uwAPB1Prescaler = 0U;
  uint32_t              uwPrescalerValue = 0U;
  uint32_t              pFLatency;

  /*Configure the TIM2 IRQ priority */
  HAL_NVIC_SetPriority(TIM2_IRQn, TickPriority ,0U);
  /* Enable the TIM2 global Interrupt */
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  /* Enable TIM2 clock */
  __HAL_RCC_TIM2_CLK_ENABLE();

  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Get APB1 prescaler */
  uwAPB1Prescaler = clkconfig.APB1CLKDivider;

  /* Compute TIM2 clock */
  if (uwAPB1Prescaler == RCC_HCLK_DIV1)
  {
    uwTimclock = HAL_RCC_GetPCLK1Freq();
  }
  else
  {
    uwTimclock = 2*HAL_RCC_GetPCLK1Freq();
  }

  /* Compute the prescaler value to have TIM2 counter clock equal to 1MHz */
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

  /* Initialize TIM2 */
  Tim2Handle.Instance = TIM2;

  /* Initialize TIMx peripheral as follow:
  + Period = [(TIM2CLK/1000) - 1]. to have a (1/1000) s time base.
  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
  + ClockDivision = 0
  + Counter direction = Up
  */
//  Tim2Handle.Init.Period = (1000000U / 1000U) - 1U;
  Tim2Handle.Init.Period = 72000000;
  Tim2Handle.Init.Prescaler = uwPrescalerValue;
  Tim2Handle.Init.ClockDivision = 0U;
  Tim2Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  Tim2Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if(HAL_TIM_Base_Init(&Tim2Handle) == HAL_OK)
  {
    /* Start the TIM time Base generation in interrupt mode */
    return HAL_TIM_Base_Start_IT(&Tim2Handle);
  }

  /* Return function status */
  return HAL_ERROR;
}

/**
  * @brief  Suspend Tick increment.
  * @note   Disable the tick increment by disabling TIM2 update interrupt.
  * @retval None
  */
void HAL_SuspendTick(void)
{
  /* Disable TIM2 update Interrupt */
  __HAL_TIM_DISABLE_IT(&Tim2Handle, TIM_IT_UPDATE);
}

/**
  * @brief  Resume Tick increment.
  * @note   Enable the tick increment by Enabling TIM2 update interrupt.
  * @retval None
  */
void HAL_ResumeTick(void)
{
  /* Enable TIM2 Update interrupt */
  __HAL_TIM_ENABLE_IT(&Tim2Handle, TIM_IT_UPDATE);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  HAL_IncTick();
}

/**
  * @brief  This function handles TIM interrupt request.
  * @retval None
  */

void PC13_led_toggle(void);

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&Tim2Handle);

  // begin user code
//  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);	// toggle LED at PC13
  PC13_led_toggle();  // see PC13_led.c
  // end user code

  __HAL_TIM_CLEAR_IT(&Tim2Handle, TIM_IT_UPDATE);
}


/**
  * @}
  */

// ********************************************************
// Copyright (c) 2022 AARI Corporation. All rights reserved
// Author Wei-Min Shen, 3-26-2022
// ********************************************************

uint32_t led_state = 0;

void PC13_led_toggle(void)
{
  volatile uint32_t* GPIOC_BSRR = (volatile uint32_t*) 0x40011010;      // Port C
  if (led_state) {
        *GPIOC_BSRR |= ((uint32_t) 1 << (13 + 0));  // LED on
        led_state = 0;
  } else {
        *GPIOC_BSRR |= ((uint32_t) 1 << (13 + 16));  // LED off
        led_state = 1;
  }
}

void MX_TIM_Init(void)
{
	HAL_TIM_Base_Start_IT(&Tim2Handle);	// this starts the timer2
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
