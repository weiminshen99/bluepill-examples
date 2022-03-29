// Copyright (c) 2022 AARI Corporation. All rights reserved
// Author Wei-Min Shen, 3-11-2022
//
// Document TRM: https://www.st.com/resource/en/reference_manual/cd00171190.pdf

#include <stdio.h>

#define BIT(x) ((uint32_t) 1 << (x))

void delay(uint16_t ms)
{
        volatile int count = 1000 * ms;
        while ( count-- );
}

// Objective: the minimal code to use registers to turn on LED on PC13
// 1. Enable GPIO Port C
// 2. Set Mode of PC13 to Output_2
// 3. Set PC13=0

void main() {

  // 1. Enable GPIO Port C
  // RCC registers, TRM section 7.3: RCC_Base 0x40021000 + APB2ENR_offset 0x18
  volatile uint32_t* RCC_APB2ENR = (volatile uint32_t*) 0x40021018;
  *RCC_APB2ENR |= (0b1<<4);   // set bit4=1 to enable Port C

  // 2. Set Mode of PC13 to Output_2
  // GPIO registers, TRM section 9.2: GPIO_Base 0x40011000 + CRH_offset 0x04
  volatile uint32_t* GPIOC_CRH = (volatile uint32_t*) 0x40011004;
  *GPIOC_CRH &= ~(0b1111<<20);  // clear bits 23..20
  *GPIOC_CRH |=  (0b0110<<20);  // set them to 0110 for Mode Output 2

  // 3. Set PC13=0
  // GPIO_Base 0x40011000, BSRR_offset 0x10
  volatile uint32_t* GPIOC_BSRR = (volatile uint32_t*) 0x40011010;

  while(1)
    {
	delay(250);
	*GPIOC_BSRR |= BIT(13 + 0);  // turn LED on
	delay(250);
	*GPIOC_BSRR |= BIT(13 + 16); // turn LED off
    }
}

