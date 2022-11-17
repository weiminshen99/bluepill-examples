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

void TIM2_IT_Init();
void TIM2_PWM1_Init();

// this is needed for HAL_Delay()
void SysTick_Handler(void)	// this SysTick Timer is running at _MHZ
{
  HAL_IncTick();                  	// at Hardware Abstraction Level (HAL), OR
  //extern int32_t uwTick; uwTick++;	// equivelently, do it at the hardware level
}

//
// System Clock Configuration
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

//  TIM1_Init();

//  TIM2_IT_Init();

  TIM2_PWM1_Init();

  if (HAL_InitTick(0) != HAL_ERROR)
  {
	// means the timers are started successfully
  }

  PC13_led_init();	// needed for toggle PC13/LED

  TIM2->CCR1 = 1000;

  while(1)
    {
/*    while(CH1_DC < 65535)
      {
	TIM2->CCR1 = CH1_DC;
    	CH1_DC += 70;
    	HAL_Delay(1);
      }
    while(CH1_DC > 0)
    	{
    	    TIM2->CCR1 = CH1_DC;
    	    CH1_DC -= 70;
    	    HAL_Delay(1);
    	}
*/
	HAL_Delay(500);	// it is about 500 ms
	//PC13_led_toggle();
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	TIM2->CCR1 = ( TIM2->CCR1==0 ? 1000 : 0 ); // toggle CCR1
    }
}
