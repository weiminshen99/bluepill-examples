// Define to prevent recursive inclusion
#ifndef TIM1_PWM1_H
#define TIM1_PWM1_H

#include "stm32f1xx_hal.h"

#include "defines.h"

void TIM1_and_Pins_Init();

void TIM1_PWM1_Init();
void TIM1_PWM1_GPIO_Init();

void Error_Handler();


#endif

