###############################################################################
# Makefile for the project panelcontroller
###############################################################################

## General Flags.
TARGET         = panelcontroller
CSRC           = main.c utils.c uart.c handler.c twi.c ff.c mmc.c rtc.c timer.c eeprom_driver.c
ASRC           = xitoa.S
MCU_TARGET     = atxmega128a1
F_CPU          = 32000000
OPTIMIZE       = -Os -mcall-prologues
DEFS           = -DF_CPU=$(F_CPU)UL
LIBS           =
DEBUG          = dwarf-2
FLASH_SERIALPORT = /dev/ttyUSB0
FLASH_BAUDRATE   = 115200


## Programs.
CC             = avr-gcc
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE           = avr-size

## Options common to compile, link and assembly rules.
COMMON = -mmcu=$(MCU_TARGET)

## Compile options common for all C compilation units.
CFLAGS         = $(COMMON) $(OPTIMIZE) 
CFLAGS        += -g$(DEBUG) -std=gnu99 $(DEFS)



## Assembly specific flags.
ASMFLAGS       = $(COMMON) 
ASMFLAGS      += -I. -x assembler-with-cpp -Wa,-adhlns=$(<:.S=.lst),-gstabs 


## Linker flags.
LDFLAGS        = -Wl,-Map,$(TARGET).map


## Objects that must be built in order to link.
OBJ            = $(CSRC:.c=.o) $(ASRC:.S=.o)


## Build
all: $(TARGET).elf lst text size


## Compile
$(TARGET).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.S
	$(CC) -c $(ASMFLAGS) $< -o $@


lst:  $(TARGET).lst
%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@


text: hex
hex:  $(TARGET).hex

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

size: $(TARGET).elf
	$(SIZE) -C --mcu=$(MCU_TARGET) $(TARGET).elf
    

## Clean target
.PHONY: clean flash
clean:
	rm -rf *.o $(TARGET).elf *.eps *.bak *.a
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)
	rm -rf $(TARGET).hex


flash:
	avrdude -p $(MCU_TARGET) -P $(FLASH_SERIALPORT) -b $(FLASH_BAUDRATE) -c avr109 -e -U flash:w:$(TARGET).hex

    