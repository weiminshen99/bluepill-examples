
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

***

Some of these projects target a Maple board (or Olimexduino).

I used to link these projects and loaded them to a different address.
(0x08005000 rather than 0x08000000).
I don't do this any more.  I build them at 0x80000000 just like all
the other projects.

I now use the STLINK for the Maple, as well as the pill.

I used to use the USB loader along with dfu-util for these.
The USB loader resided at 0x80000000, so my code had to go
up higher (0x80005000 worked).  I burned my bridges and
just overwrote the USB loader (but I did save a copy of
that loader, just in case).

***

Many of these projects require/expect a serial port.

This requires a USB to serial gadget connected to PB9 and PB10
Connect PA9 to RX
Connect PA10 to TX
also connect ground.

***

Here is a list of projects from 2017 ---

1. loader - host side program to talk to serial bootloader.
1. serial_boot - disassembly of the boot loader

1. blink1 - very basic LED blinking
2. blink1b - minor code cleanup on the above
3. blink_ext - blink external LED on A0
4. blink2 - more code reorganization and file grouping
5. blink_maple - blink LED on PA5 rather than PC13  --M
6. interrupt - do timer interrupts
6. lcd - drive a 16x2 LCD module using i2c

6. memory - figure out how to handle bss and initialized data --B
7. i2c_maple - use libmaple for i2c DAC --B
8. adc_serial - work with F103 adc --P

9. serial1 - first working serial (uart) driver
9. lithium1 - lithium ion battery tester --P

Note that blink_maple is the only demo built to run on the Maple board.

The letters are: M = links for Maple, B = zeros BSS, P = includes printf

***

The above work was done in 2017, I am beginning new work in 2023,
focusing on learning about low level USB

9. usb1 - get barely started working with USB (2017) --P
2. usb_papoon - the papoon_usb framework and a working echo! -BP
3. usb_papoon2 - the papoon_usb framework using interrupts -BP
4. usb_baboon - papoon2 running on the Maple board -BPM

The above are just stepping stones to the following, which is
wht you really want.

1. usb - the final USB serial project  -BPM

