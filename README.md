
================================================
History:

03-xx-2023: Added HAL_examplesc

11-19-2022: WMS added the PWM code for working examples

3-29-2022: WMS added a few more examples in the collection.  -- WMS. 3-29-2022


================================================
Descriptions:

START_HERE: I made the minimal code to blink LED on PC13. Check it out!

HAL_Examples: has all the useful examples based on HAL.

Other examples are from:

https://github.com/trebisky/stm32f103

https://github.com/trebisky/stm32f411

https://github.com/trebisky/libmaple-unwired

https://github.com/cesanta/stm32-bluepill

http://cholla.mmto.org/stm32/

https://github.com/dwelch67/stm32_samples/tree/master/STM32F103C8T6

Note that many of the early examples have a naive linker script
(lds file) which does not initialize BSS or global variables at all.
Later examples require this.  The presence of startup.c should be
an indicator of more sophisticated and proper treatement.
