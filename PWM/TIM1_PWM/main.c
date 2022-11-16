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

void GPIO_Init_for_PWM(); // see tim1_pwm.c
void TIM1_PWM1_Init(); 	  // see tim1_pwm.c

// this is needed for HAL_Delay()
void SysTick_Handler(void)	// this SysTick Timer is running at _MHZ
{
  HAL_IncTick();                  	// at Hardware Abstraction Level (HAL), OR
  //extern int32_t uwTick; uwTick++;	// equivelently, do it at the hardware level
}

//
// System Clock Configuration
//
//
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

//===========================
//	PC13 LED
//===========================

volatile uint32_t* GPIOC_BSRR = (volatile uint32_t*) 0x40011010; // Port C
volatile uint32_t led_state = 0;

void PC13_led_init(void)
{
  volatile uint32_t* RCC_APB2ENR = (volatile uint32_t*) 0x40021018;     // RCC
  volatile uint32_t* GPIOC_CRH =  (volatile uint32_t*) 0x40011004;      // Port C
  *RCC_APB2ENR |= (0b1<<4);     // set bit4=1 to enable Port C
  *GPIOC_CRH &= ~(0b1111<<20);  // clear PC13, bits 23..20
  *GPIOC_CRH  |=  (0b0110<<20);  // set the bits to 0110 for Mode Output 2
  *GPIOC_BSRR |= ((uint32_t) 1 << (13 + 0)); // turn off PC13/LED
}

void PC13_led_toggle(void)
{
  if (led_state) { // make PC13/LED OFF
        *GPIOC_BSRR |= ((uint32_t) 1 << (13 + 0));
        led_state = 0;
  } else { // make it ON
        *GPIOC_BSRR |= ((uint32_t) 1 << (13 + 16));
        led_state = 1;
  }
}



// ======= TIM2 ====================================================

TIM_HandleTypeDef tim2_handle;

HAL_StatusTypeDef TIM2_Init()
{
  // Initialize TIM2
  tim2_handle.Instance = TIM2;
  tim2_handle.Init.Period = 72000000;
  tim2_handle.Init.Prescaler = 1;	// xN will slow N times
  tim2_handle.Init.ClockDivision = 0U;
  tim2_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  tim2_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&tim2_handle)==HAL_ERROR)
     return HAL_ERROR;

  // Configure the TIM2 IRQ priority
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  // Enable the TIM2 global Interrupt
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  // Enable TIM2 clock
  __HAL_RCC_TIM2_CLK_ENABLE();
  // Starts the interrupts of tim2
  return HAL_TIM_Base_Start_IT(&tim2_handle);
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&tim2_handle);
  // begin user code
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	// toggle LED at PC13
  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);	// turn OFF LED at PC13
  // end user code
  __HAL_TIM_CLEAR_IT(&tim2_handle, TIM_IT_UPDATE);
}


// ======= TIM1 ====================================================
TIM_HandleTypeDef tim1_handle;

HAL_StatusTypeDef TIM1_Init()
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

  PC13_led_init();	// needed for toggle PC13/LED

  GPIO_Init_for_PWM();
  TIM1_PWM1_Init();

//  TIM1_Init();

//  TIM2_Init();

  if (HAL_InitTick(0) != HAL_ERROR)
  {
	// means the timers are started successfully
  }

  while(1)
  {
/*
        while(CH1_DC < 65535)
        { // increase PWM duty cycles
          TIM1->CCR1 = CH1_DC;
          CH1_DC += 70;
          HAL_Delay(1);
        }
        while(CH1_DC > 0)
        { // decrease PWM duty cycles
            TIM1->CCR1 = CH1_DC;
            CH1_DC -= 70;
            HAL_Delay(1);
        }
*/
        HAL_Delay(100); // is it about 500 ms?
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // PC13 LED
    }
}


