/*
* Copyright (C) 2022-2023 AARICO Corporation <weiminshen99@gmail.com> 
*
* This file is branched out and modified from the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
//#include "hd44780.h"

void SystemClock_Config(void);

extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart2;

// the followings are from bldc.c
extern uint8_t enable;
extern int pwm_res;

typedef struct{
	uint16_t start_of_frame;
	int16_t  steer;
	int16_t  speed;
	uint16_t checksum;
} Serialcommand;

volatile Serialcommand command;

uint8_t button1, button2;

//int steer; // global variable for steering. -1000 to 1000
//int speed; // global variable for speed. -1000 to 1000

extern volatile int pwml;  // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;  // global variable for pwm right. -1000 to 1000
extern volatile int weakl; // global variable for field weakening left. -1000 to 1000
extern volatile int weakr; // global variable for field weakening right. -1000 to 1000

extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable; // global variable for motor enable

extern volatile uint32_t timeout; // global variable for timeout
extern float batteryVoltage; // global variable for battery voltage

uint32_t inactivity_timeout_counter;
uint32_t main_loop_counter = 0;

int32_t motor_test_direction = 1;

extern uint8_t nunchuck_data[6];
#ifdef CONTROL_PPM
extern volatile uint16_t ppm_captured_value[PPM_NUM_CHANNELS+1];
#endif

int milli_vel_error_sum = 0;

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

  if (hall_a==1 && hall_b==0 && hall_c==0) {		// if hall is 100
     *u = pwm;  *v = 0;     *w = minus_pwm;		// 	A -> C
  } else if (hall_a==1 && hall_b==1 && hall_c==0) { 	// if hall is 110
     *u = pwm;  *v = minus_pwm;  *w = 0;		// 	A -> B
  } else if (hall_a==0 && hall_b==1 && hall_c==0) { 	// if hall is 010
     *u = 0;  *v = minus_pwm;  *w = pwm;		// 	C -> B
  } else if (hall_a==0 && hall_b==1 && hall_c==1) {	// if hall is 011
     *u = minus_pwm;  *v = 0;  *w = pwm;		//	C -> A
  } else if (hall_a==0 && hall_b==0 && hall_c==1) {	// if hall is 001
     *u = minus_pwm;  *v = pwm;  *w = 0;		//	B -> A
  } else if (hall_a==1 && hall_b==0 && hall_c==1) {	// if hall is 101
     *u = 0;  *v = pwm;  *w = minus_pwm;		//	B -> C
  } else {						// otherwise
     *u = 0;  *v = 0;  *w = 0;				// do nothing
  }
}

int main(void) {

  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();

  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();

  /*
  MX_ADC1_Init();
  MX_ADC2_Init();

  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, 1);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  for (int i = 8; i >= 0; i--) {
    buzzerFreq = i;
    HAL_Delay(100);
  }
  buzzerFreq = 0;

  HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  */

  int ur, vr, wr;
  uint8_t hall_ur, hall_vr, hall_wr;

  enable = 1;  // enable motors, see bldc.c

  while(1) {

    pwmr = 900;	// speed

    // read hall sensors
    //hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
    //hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
    //hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

    get_next_hall_readings(main_loop_counter%6, &hall_ur, &hall_vr, &hall_wr);

    //hall_ur=0; hall_vr=1; hall_wr=0;

    hall_to_PWM(pwmr, hall_ur, hall_vr, hall_wr, &ur, &vr, &wr);

    /*
    RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res/2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res/2, 10, pwm_res-10);
    RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res/2, 10, pwm_res-10);
    */

    if (ur != 0) {
    	HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_1);
    	HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_1);
	RIGHT_TIM->RIGHT_TIM_U = CLAMP(1000+ur, 10, 2000);
    } else {
    	HAL_TIM_PWM_Stop(&htim_right, TIM_CHANNEL_1);
    	HAL_TIMEx_PWMN_Stop(&htim_right, TIM_CHANNEL_1);
    }

    if (vr != 0) {
    	HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_2);
    	HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_2);
	RIGHT_TIM->RIGHT_TIM_V = CLAMP(1000+vr, 10, 2000);
    } else {
    	HAL_TIM_PWM_Stop(&htim_right, TIM_CHANNEL_2);
    	HAL_TIMEx_PWMN_Stop(&htim_right, TIM_CHANNEL_2);
    }

    if (wr != 0) {
    	HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_3);
    	HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_3);
	RIGHT_TIM->RIGHT_TIM_W = CLAMP(1000+wr, 10, 2000);
    } else {
    	HAL_TIM_PWM_Stop(&htim_right, TIM_CHANNEL_3);
    	HAL_TIMEx_PWMN_Stop(&htim_right, TIM_CHANNEL_3);
    }

    HAL_Delay(20); // the 10ms duration for the current PWM state


    main_loop_counter += 1;
    timeout++;

    if (main_loop_counter % 100 == 0) {
	HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
