
TIM2_PWM1 runs a buzzer at Pin A0 (default value is 1000);

ADC1 reads Channel7 at Pin A7 and puts the value into AD_RES (uint32_t);

AD_RES is used to change the volume of the buzzer 
	TIM2->CCR1 = (AD_RES<<4);

