
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "tim1_pwm1.h"

void SystemClock_Config(void);
void LED_Init();
void TIM1_and_Pins_Init();

extern TIM_HandleTypeDef htim_right;	// timer1 handle

void get_next_hall_readings(int index, uint8_t *hall_a, uint8_t *hall_b, uint8_t *hall_c)
{
     switch(index) {
        case 0: *hall_a=1; *hall_b=0; *hall_c=0; break;
        case 1: *hall_a=1; *hall_b=1; *hall_c=0; break;
        case 2: *hall_a=0; *hall_b=1; *hall_c=0; break;
        case 3: *hall_a=0; *hall_b=1; *hall_c=1; break;
        case 4: *hall_a=0; *hall_b=0; *hall_c=1; break;
        case 5: *hall_a=1; *hall_b=0; *hall_c=1; break;
        default: *hall_a=1; *hall_b=0; *hall_c=0;
     }
}

void hall_to_PWM(int pwm, int hall_a, int hall_b, int hall_c, int *u, int *v, int *w) 
{
  int minus_pwm = -pwm;;
  //  minus_pwm = -1000; // testing

  if (hall_a==1 && hall_b==0 && hall_c==0) {            // if hall is 100
     *u = pwm;  *v = 0;     *w = minus_pwm;             //      A -> C
  } else if (hall_a==1 && hall_b==1 && hall_c==0) {     // if hall is 110
     *u = pwm;  *v = minus_pwm;  *w = 0;                //      A -> B
  } else if (hall_a==0 && hall_b==1 && hall_c==0) {     // if hall is 010
     *u = 0;  *v = minus_pwm;  *w = pwm;                //      C -> B
  } else if (hall_a==0 && hall_b==1 && hall_c==1) {     // if hall is 011
     *u = minus_pwm;  *v = 0;  *w = pwm;                //      C -> A
  } else if (hall_a==0 && hall_b==0 && hall_c==1) {     // if hall is 001
     *u = minus_pwm;  *v = pwm;  *w = 0;                //      B -> A
  } else if (hall_a==1 && hall_b==0 && hall_c==1) {     // if hall is 101
     *u = 0;  *v = pwm;  *w = minus_pwm;                //      B -> C
  } else {                                              // otherwise
     *u = 0;  *v = 0;  *w = 0;                          // do nothing
  }
}

int main(void)
{
    HAL_Init();

    SystemClock_Config();

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    LED_Init();

    TIM1_and_Pins_Init();  // three channels, CH1, CH2, CH3

    int pwm = 900; // speed
    int main_loop_counter = 0;
    uint8_t hall_a, hall_b, hall_c;
    int u, v, w;

    while(1) {

    // read hall sensors
    //hall_a = !(HALL_U_PORT->IDR & HALL_U_PIN);
    //hall_b = !(HALL_V_PORT->IDR & HALL_V_PIN);
    //hall_c = !(HALL_W_PORT->IDR & HALL_W_PIN);

    get_next_hall_readings(main_loop_counter%6, &hall_a, &hall_b, &hall_c);

    hall_to_PWM(pwm, hall_a, hall_b, hall_c, &u, &v, &w);

    /*
    MOTOR_TIM->MOTOR_TIM_U = CLAMP(u + pwm_res/2, 10, pwm_res-10);
    MOTOR_TIM->MOTOR_TIM_V = CLAMP(v + pwm_res/2, 10, pwm_res-10);
    MOTOR_TIM->MOTOR_TIM_W = CLAMP(w + pwm_res/2, 10, pwm_res-10);
    */

    if (u != 0) {
        HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_1);
        MOTOR_TIM->MOTOR_TIM_U = CLAMP(1000+u, 10, 2000);
    } else {
        HAL_TIM_PWM_Stop(&htim_right, TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(&htim_right, TIM_CHANNEL_1);
    }

    if (v != 0) {
        HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_2);
        MOTOR_TIM->MOTOR_TIM_V = CLAMP(1000+v, 10, 2000);
    } else {
        HAL_TIM_PWM_Stop(&htim_right, TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(&htim_right, TIM_CHANNEL_2);
    }

   if (w != 0) {
        HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_3);
        MOTOR_TIM->MOTOR_TIM_W = CLAMP(1000+w, 10, 2000);
    } else {
        HAL_TIM_PWM_Stop(&htim_right, TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(&htim_right, TIM_CHANNEL_3);
    }

    HAL_Delay(20); // the duration for the each PWM state

    main_loop_counter += 1;

    if (main_loop_counter % 100 == 0) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
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
void LED_Init(void)
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
