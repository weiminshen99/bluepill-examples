#include "stm32f1xx_hal.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

void Error_Handler(void);
void SystemClock_Config(void);
static void ADC1_Init(void);
static void ADC1_GPIO_Init(void);
//static void MX_DMA_Init(void);
static void TIM2_Init(void);
static void MX_TIM3_Init(void);
static void LED_GPIO_Init(void);


uint32_t AD_RES = 0;

// ===========================================
int main(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_Init();
    SystemClock_Config();

    TIM2_Init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// you can listen or see led on this channel

    LED_GPIO_Init();
    MX_TIM3_Init();

    ADC1_Init();
    ADC1_GPIO_Init();

    //MX_DMA_Init();

    HAL_ADCEx_Calibration_Start(&hadc1);

    //
    // Demonstrate the four methods to control/get/use data from ADC1
    //
    while (0) // Method-1: Poll data from ADC by yourself
    {
        HAL_ADC_Start(&hadc1);			// Start ADC1 normally
    	HAL_ADC_PollForConversion(&hadc1, 1); 	// Poll ADC1_CH1 & TimeOut=1ms
    	AD_RES = HAL_ADC_GetValue(&hadc1);	// Get the data from ADC1
    	TIM2->CCR1 = (AD_RES<<4);		// Use the data to control TIM2_CH1_PWM
    	HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }

    while (0) // Method-2: ADC1 generates an interrupt upon completion of conversion
    {
        HAL_ADC_Start_IT(&hadc1);  // Start ADC1 with Interrupt
        // When ADC1 completes a coversion, it will generate an interrupt which will call ADC1_2_IRQHandler
        TIM2->CCR1 = (AD_RES<<4);
        HAL_Delay(100);
    }

    while (0) // Method-3: ADC1->DMA1, DMA1 generates an interrupt upon completion
    {
        //HAL_ADC_Start_DMA(&hadc1, &AD_RES, 1); // start ADC1 with DMA, not working
        //HAL_DMA_Start_IT(&hdma_adc1, &hadc1, &AD_RES, 1); // start ADC1+DMA, not working
        HAL_ADC_Start(&hadc1); // configure DMA1_Channel1 inside ADC1_Init() and start ADC1 normally
	// for each interrupt from DAM1_Channel1, use DAM1_Channel1_IRQHandler to access the data
        HAL_Delay(100);
    }

    // Method-4: Use TIM3 to trigger ADC1
    HAL_ADC_Start(&hadc1); 	// need to start ADC1 only once, then
    HAL_TIM_Base_Start(&htim3); // TIM3 will trigger ADC1. Note: No TIM3, no ADC1.
    // Trying to use TIM2 or TIM1 to trigger ADC1, still testing
    //HAL_TIM_Base_Start(&htim2);
    //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    while (1)
    {   // In the loop, no need to call HAL_ADC_Start() again,
	// because ADC1 is triggered automatically and periodically by TIM3
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // show alive
    }
}

// This callback works Method-2 above
// the interrupts are generated from HAL_ADC_Start_IT(&hadc1)
void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&hadc1);	// this interrupt indicates that ADC1 has completed conversion
    AD_RES = HAL_ADC_GetValue(&hadc1);		// now we can get data from ADC1 to AD_RES
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	// show here
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);	// clear this interrupt flag
}

// This callback works for Method-3
// the interrupts are generated by DMA1_Channel1 configured inside ADC1_Init()
void DMA1_Channel1_IRQHandler(void)
{   // this interrupt indicates that DMA1_Channel1 already transfers data from ADC1 to AD_RES
    TIM2->CCR1 = (AD_RES<<4);	// we can now use AD_RES as we wish (i.e., control TIM2_CH1_PWM)
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // show here
    DMA1->IFCR = DMA_IFCR_CTCIF1; // clear this interrupt flag for DMA1_Channel1
}



/* this is not working
// For ADC -> Interrupt
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Read & Update The ADC Result
    AD_RES = HAL_ADC_GetValue(&hadc1);
>    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
*/

/*
static void MX_DMA_Init(void)
{
    // DMA controller clock enable
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_DMA_ENABLE(&hdma_adc1);

//    HAL_DMA_Start_IT(&hdma_adc1, &hadc1, &AD_RES, 1);

//   __HAL_LINKDMA(&hadc1, &hdma_adc1);

    // DMA interrupt init
    // DMA1_Channel1_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}
*/

//
// For ADC1 -> DMA, this callback does not work for HAL_ADC_Start_DMA(__), don't know why
//
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
    // Conversion Complete & DMA Transfer Complete As Well
    // So The AD_RES Is Now Updated & Let's Move IT To The PWM CCR1
    // Update The PWM Duty Cycle With Latest ADC Conversion Result
//    TIM2->CCR1 = (AD_RES<<4);
//    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//}

static void MX_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 72000000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  //TIM3->ARR = 240000; // ARR: Auto reload register, determine how often to send trigger

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
	Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    	Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    	Error_Handler();
  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
//  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
//  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  //__HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();  // To be triggered by T3_TRGO

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;	// <== PA7 input
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_ADC_ENABLE(&hadc1);

  // when ADC1 is started with interrupt as HAL_ADC_Start_IT(_)
  // the following two lines enables the interrupt to be generated
  // HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  // HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

  //
  // magically, the following links adc1 to DMA1_Cheannel1 which generates interrupt
  //
  hadc1.Instance->CR2 |= ADC_CR2_DMA | ADC_CR2_TSVREFE;
  __HAL_RCC_DMA1_CLK_ENABLE();
  DMA1_Channel1->CCR   = 0;
  DMA1_Channel1->CNDTR = 1;	// = length of data to be transferred
  DMA1_Channel1->CPAR  = (uint32_t) & (ADC1->DR); // from adc1
  DMA1_Channel1->CMAR  = (uint32_t) & (AD_RES);	  // to AD_RES in memory
  DMA1_Channel1->CCR   = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE;
  DMA1_Channel1->CCR  |= DMA_CCR_EN;
  // enable interrupt from DMA1_Channel1
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  __HAL_RCC_TIM2_CLK_ENABLE();

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // or UPDATE to trigger ADC1
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  // Try TIM2 CH2 to trigger ADC1
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure PA0 for output
  // 	TIM2_CH1 ==> PA0
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void ADC1_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void LED_GPIO_Init(void)
{  // For PC13 LED
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/