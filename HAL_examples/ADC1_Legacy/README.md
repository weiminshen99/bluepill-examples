
Wei-Min Shen, 2023-3-10

The code in this direction is from the website:

https://deepbluembedded.com/stm32-adc-read-example-dma-interrupt-polling/

It works on bluepill for me, but somehow the boot1 would be corrupted if
I load the compiled code as it is. To load again, I have to use boot0 by
moving the jumper on bluepill.

This situation has happened before as follows:
	
	../PWM_TIM2_Legacy      // cause boot1 to coorupt
	../PWM_TIMx		// without coorupting boot1

Now, I plan to make the same changes:

	../ADC1_Legacy		// cause the boot1 coorupt 
	../ADC1_new		// without cooruption (to be done) 
