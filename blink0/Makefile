# Copyright (c) AARI Corporation, All Right Served
# Author Wei-Min Shen, 3-11-2020

TARGET	= main
C_SOURCES   = main.c
ASM_SOURCES = bootstrap.s
LINK_SCRIPTS = link.ld

#TOOLS = arm-linux-gnu
TOOLS = arm-none-eabi

# Assembling with gcc makes it want crt0 at link time.
#AS = $(TOOLS)-gcc
AS = $(TOOLS)-as
# Use the -g flag if you intend to use gdb
#CC = $(TOOLS)-gcc -mcpu=cortex-m3 -mthumb
CC = $(TOOLS)-gcc -mcpu=cortex-m3 -mthumb -g

#LD = $(TOOLS)-gcc
LD = $(TOOLS)-ld.bfd
OBJCOPY = $(TOOLS)-objcopy
DUMP = $(TOOLS)-objdump -d
GDB = $(TOOLS)-gdb

OBJS = $(C_SOURCES:.c=.o) $(ASM_SOURCES:.s=.o)

all: $(TARGET).bin

$(TARGET).hex: $(TARGET).bin
	$(OBJCOPY) -I binary -O ihex --change-address 0x8000000 $< $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(TARGET).elf: $(OBJS)
	$(LD) -T $(LINK_SCRIPTS) -o $@ $(OBJS)

%.o: %.s
	$(AS) $< -o $@

%.o: %.c
	$(CC) -c $< -o $@

flash:  $(TARGET).bin
	st-flash write $< 0x08000000

gdb:    $(TARGET).elf
	st-util -p 4242 &
	$(GDB) --eval-command="target extended-remote :4242" $(TARGET).elf

clean:
	rm -rf *.{o,bin,elf,map,lst,tgz,zip,hex,dump}
