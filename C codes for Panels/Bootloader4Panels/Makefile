
MCU = atmega168
FORMAT = ihex
BOOTLOADER = panel_bl.elf
OPT = s

CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AVRDUDE = avrdude

# Name of this Makefile (used for "make depend").
MAKEFILE = Makefile

# Compiler flag to set the C Standard level.
# c89   - "ANSI" C
# gnu89 - c89 plus GCC extensions
# c99   - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

# Place -D or -U options here
CDEFS = -DF_CPU=20000000UL

# Place -I options here
CINCS =

# Compile options common for all C compilation units.
ifdef windir
CDEBUG = -gdwarf-2
else
CDEBUG = -gstabs
endif
CWARN = -Wall -Wstrict-prototypes
CTUNING = -finline-limit=10 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS = $(COMMON) $(CDEBUG) $(CDEFS) $(CINCS) -O$(OPT) $(CWARN) \
	 $(CSTANDARD) $(CEXTRA) $(CTUNING)
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += -x assembler-with-cpp -Wa,-gstabs


## Linker flags for bootloader
BL_LDFLAGS = $(COMMON)
BL_LDFLAGS +=  -Wl,--section-start=.text=0x3800 -Wl,-Map=panel_bl.map

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0

## All source and header files
CSOURCE = $(wildcard *.c)
HEADERS = $(wildcard *.h)


## Build
all: $(BOOTLOADER) panel_bl.hex panel_bl.lss bl_size

## Compile
bootloader.o: bootloader.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<





$(BOOTLOADER): bootloader.o
	 $(CC) $(BL_LDFLAGS) bootloader.o -o $(BOOTLOADER)

%.hex: %.elf
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: %.elf common/config.h
	avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@

%.lss: %.elf
	avr-objdump -h -S $< > $@


bl_size: ${BOOTLOADER}
	@echo
ifdef windir
	@avr-size -C --mcu=${MCU} ${BOOTLOADER}
else
	@avr-size ${BOOTLOADER}
endif

## Clean target
.PHONY: clean
clean:
	-rm -rf bootloader.o panel_bl.hex panel_bl.lss \
	    panel_bl.map panel_bl.elf dep

## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)


