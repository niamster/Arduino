# Makefile for ATmega16BOOT
# D.Milinevskyy, 2011-07-24

# program name should not be changed...
PROGRAM    = ATmega16BOOT

PRODUCT=atmega16

DIRAVR = /usr
DIRAVRBIN = $(DIRAVR)/bin
DIRINC = $(DIRAVR)/avr/include
DIRLIB = $(DIRAVR)/avr/lib


MCU_TARGET = atmega16
LDSECTION  = --section-start=.text=0x3800

OBJ        = $(PROGRAM).o
OPTIMIZE   = -O2

DEFS       = -DF_CPU=7372800UL -DBAUD_RATE=9600
LIBS       =

CC         = $(DIRAVRBIN)/avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g --save-temps -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) -D$(PRODUCT) $(DEFS) -I$(DIRINC)
override LDFLAGS       = -Wl,-Map,$(PROGRAM).map,$(LDSECTION)

OBJCOPY        = $(DIRAVRBIN)/avr-objcopy
OBJDUMP        = $(DIRAVRBIN)/avr-objdump
SIZE           = $(DIRAVRBIN)/avr-size

all: $(PROGRAM).elf lst text asm size

$(PROGRAM).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.s
	rm -rf *.o *.elf
	rm -rf *.lst *.map

asm: $(PROGRAM).s

%.s: %.c
	$(CC) -S $(CFLAGS) -g1 $^

lst:  $(PROGRAM).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

size: $(PROGRAM).hex
	$(SIZE) $^

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PROGRAM).hex
bin:  $(PROGRAM).bin
srec: $(PROGRAM).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
