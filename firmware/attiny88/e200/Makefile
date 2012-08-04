#
# Copyright 2009 Ettus Research LLC
#

##################################################
# Compiler
##################################################
CC = avr-gcc
OBJCOPY = avr-objcopy
STRIP = avr-strip
OBJDUMP = avr-objdump
SREC = srec_cat
CFLAGS = -O2 -std=gnu99 -fshort-enums -pedantic-errors -Wall -Werror \
	-Wstrict-prototypes -Wmissing-prototypes -Wcast-align -Wshadow

#-D IO_DEBUG

##################################################
# Files
##################################################
HDRS =
SRCS =  main.c io.c spi.c power.c ltc3675.c
TARGET = main

##################################################
# Device
##################################################
MMCU = attiny88
PROGRAMMER = avrisp2
PORT = usb
AVRDUDE = avrdude -p $(MMCU) -c $(PROGRAMMER) -P $(PORT)

##################################################
# Global Targets
##################################################
all: $(TARGET).hex

clean:
	$(RM) *.o *.elf *.hex

install: all
	$(AVRDUDE) -U flash:w:$(TARGET).hex:i

##################################################
# Dependency Targets
##################################################
fuses.hex: $(TARGET).elf
	$(OBJCOPY) -j .fuse -O ihex $< $@ --change-section-lma .fuse=0

lfuse.hex: fuses.hex
	$(SREC) $< -Intel -crop 0x00 0x01 -offset  0x00 -O $@ -Intel

hfuse.hex: fuses.hex
	$(SREC) $< -Intel -crop 0x01 0x02 -offset -0x01 -O $@ -Intel

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -R .eeprom -R .fuse -O ihex $< $@

$(TARGET).elf: $(SRCS:.c=.o)
	$(CC) -mmcu=$(MMCU) $^ -o $@

%.o: %.c $(HDRS) Makefile
	$(CC) -mmcu=$(MMCU) -c $< -o $@ $(CFLAGS)
