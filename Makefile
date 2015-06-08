# Makefile for ATmegaBOOT
# E.Lins, 18.7.2005
# $Id: Makefile,v 1.4 2008/10/05 17:41:43 cvs Exp $
#
# Instructions
#
# To build the bootloader for the Diecimila:
# make diecimila
#
# To build the bootloader for the NG/Mini:
# make ng
#
# To build the bootloader for the LilyPad:
# make lilypad
#
# To burn the bootloader:
# make TARGET=diecimila isp
# make TARGET=ng isp
# make TARGET=lilypad isp
# make TARGET=pro8 isp

# program name should not be changed...
PROGRAM    = ATmegaBOOT_xx8

# enter the parameters for the avrdude isp tool
ISPTOOL	   = stk500v2
ISPTOOL	   = usbtiny
ISPPORT	   = usb
ISPPORT	   = /dev/ttyISP0
ISPSPEED   = -b 115200 -B 1

MCU_TARGET = atmega168
LDSECTION  = --section-start=.text=0x3800

# the efuse should really be 0xf8; since, however, only the lower
# three bits of that byte are used on the atmega168, avrdude gets
# confused if you specify 1's for the higher bits, see:
# http://tinker.it/now/2007/02/24/the-tale-of-avrdude-atmega168-and-extended-bits-fuses/
#
# similarly, the lock bits should be 0xff instead of 0x3f (to
# unlock the bootloader section) and 0xcf instead of 0x0f (to
# lock it), but since the high two bits of the lock byte are
# unused, avrdude would get confused.
ISPFUSES    = avrdude -c $(ISPTOOL) -p m168 -P $(ISPPORT) $(ISPSPEED) -e -u -U lock:w:0x3f:m -U efuse:w:0x00:m -U hfuse:w:0x$(HFUSE):m -U lfuse:w:0x$(LFUSE):m
ISPFLASH    = avrdude -c $(ISPTOOL) -p m168 -P $(ISPPORT) $(ISPSPEED) -U flash:w:$(PROGRAM)_$(TARGET).hex -U lock:w:0x0f:m

STK500 = "C:\Program Files\Atmel\AVR Tools\STK500\Stk500.exe"
STK500-1 = $(STK500) -e -d$(MCU_TARGET) -pf -vf -if$(PROGRAM)_$(TARGET).hex -lFF -LFF -f$(HFUSE)$(LFUSE) -EF8 -ms -q -cUSB -I200kHz -s -wt
STK500-2 = $(STK500) -d$(MCU_TARGET) -ms -q -lCF -LCF -cUSB -I200kHz -s -wt


OBJ        = $(PROGRAM).o
OPTIMIZE   = -O2

DEFS       = 
LIBS       =

CC         = avr-gcc

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all:

adaboot328: TARGET = adaboot328
adaboot328: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>2' '-DNUM_LED_FLASHES=3'  '-DWATCHDOG_MODS'
adaboot328: $(PROGRAM)_adaboot328.hex
adaboot328: AVR_FREQ = 16000000L 
adaboot328: MCU_TARGET = atmega328p
adaboot328: LDSECTION  = --section-start=.text=0x7800

ng: TARGET = ng
ng: CFLAGS += '-DMAX_TIME_COUNT=F_CPU>>1' '-DNUM_LED_FLASHES=3'
ng: $(PROGRAM)_ng.hex
ng: AVR_FREQ = 16000000L

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -DF_CPU=$(AVR_FREQ) $(DEFS)
override LDFLAGS       = -Wl,$(LDSECTION)
#override LDFLAGS       = -Wl,-Map,$(PROGRAM).map,$(LDSECTION)

isp328: $(PROGRAM)_$(TARGET).hex
	avrdude -c $(ISPTOOL) -p m328p -P $(ISPPORT) $(ISPSPEED) -e -u -U lock:w:0x3f:m -U efuse:w:0x05:m -U hfuse:w:0xDA:m -U lfuse:w:0xFF:m
	avrdude -c $(ISPTOOL) -p m328p -P $(ISPPORT) $(ISPSPEED) -U flash:w:$(PROGRAM)_$(TARGET).hex -U lock:w:0x0f:m

isp: $(PROGRAM)_$(TARGET).hex
	$(ISPFUSES)
	$(ISPFLASH)

isp-stk500: $(PROGRAM)_$(TARGET).hex
	$(STK500-1)
	$(STK500-2)

%.elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o *.elf *.lst *.map *.sym *.lss *.eep *.srec *.bin *.hex

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
