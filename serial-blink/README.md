===== HARDWARE =====

for programming:
MAC<-(usb)->[STLINK/V2]<-(swd)->[Bluepill/STM32F103C8]<-(pc13)->LED

for communication:
MAC<-(usb)->[USB-Serial-Cable]<-(uart1)->[Bluepill/STM32F103C8]

===== SOFTWARE (see Makefile) =====

arm-none-eabi-{gcc,ldf,as}
ST-Flash utility (https://github.com/texane/stlink)

===== Operation Procedure =====

0. Type "make" to make $(TARGET).{elf,bin}
1. Type "make flash" to load into Bluepill via STLINK V2 dongle
2. Now, you shall see the LED blinking on Bluepill
3. Replace STLINK by a USB-Serial cable to USART1 on Bluepill
   MAC <-> [USB-Serial-Cable] <-(USART1)-> [PA9,PA10,G,5V] Bluepill
5. On MAC, run "ls /dev/cu.usbserial-*" to identify the serial port
6. On MAC, run "screen /dev/cu.usbserial-1410 115200"
   or  "minicom -b 115200 -o -D /dev/cu.usbserial-1410"
7. You should see the printings on the terminal
8. You can type in a number [0..9] to change the blink period
9. Press the reset button on Bluepill to continue (to be improved)

===== THE END =====

