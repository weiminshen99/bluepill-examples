This is the minimum LED code for STM32

===== HARDWARE =====

[MAC/PC] <-(usb)-> [STLINK/V2] <-(swd)-> [Bluepill/STM32F103C8] <-(PC13)-> LED

===== SOFTWARE (see Makefile) =====

arm-none-eabi-{gcc,ldf,as}
ST-Flash utility (https://github.com/texane/stlink)

===== Operation Procedure =====

0. Type "make" to make $(TARGET).{elf,bin}
1. Type "make flash" to load into STM32/Bluepill via [STLINK V2]
2. Now, you shall see the LED is blinking.

===== THE END =====

