
THis respoository does the PWM generation using HAL programming:

TIM1 ------> Channel-1 -----> PA8  ---> PWM (with deadtime)
       |---> Channel-1N ----> PB13 ---> PWM (with deadtime)


TIM2 ------> Channel-1 -----> PA0  ---> PWM

In main.c, control the PWM by TIM1->CCR1=v, where v is in [0..65355]


