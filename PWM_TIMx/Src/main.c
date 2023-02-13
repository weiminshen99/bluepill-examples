
#include "stm32f1xx_hal.h"
#include "tim1_pwm1.h"
#include "tim2_pwm1.h"

void SystemClock_Config(void);
void GPIO_Init();

int main(void)
{
    int32_t CH1_DC = 0;
    HAL_Init();
    SystemClock_Config();

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE(); // this must be in the main(). Why?
    __HAL_RCC_TIM2_CLK_ENABLE(); // this must be in the main(). Why?

    GPIO_Init();

    TIM2_PWM1_Init();
    TIM2_PWM1_GPIO_Init();

    TIM1_PWM1_Init();
    TIM1_PWM1_GPIO_Init();

//    TIM1_and_Pins_Init();

    while (1)
    {
    	while(CH1_DC < 65535)
    	{
    		TIM1->CCR1 = CH1_DC;
    		TIM1->CCR2 = CH1_DC;
    		TIM1->CCR3 = CH1_DC;

    		TIM2->CCR1 = CH1_DC;

    		CH1_DC += 70;
    		HAL_Delay(1);
    	}
    	//CH1_DC = 65535;
    	while(CH1_DC > 0)
    	{
    		TIM1->CCR1 = CH1_DC;
    		TIM1->CCR2 = CH1_DC;
    		TIM1->CCR3 = CH1_DC;

    	    	TIM2->CCR1 = CH1_DC;

    	    	CH1_DC -= 70;
    	    	HAL_Delay(1);
    	}
    	//CH1_DC = 0;

	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}


/**
  * @brief SysTick Handler
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();	// needed for HAL_Delay
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

// ============================================
void GPIO_Init(void)
{  // For PC13 LED
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}


// ===========================================
void Error_Handler()
{
}
