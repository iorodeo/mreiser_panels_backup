###############################################################################
# Makefile for the project BootloaderMkII_gcc
###############################################################################

## General Flags.
TARGET           = BootloaderMkII_gcc
CSRC             = ../eeprom_driver.c ../main.c ../serial.c
ASRC             = ../sp_driver.S
MCU_TARGET       = atxmega128a1
F_CPU            = 2000000
OPTIMIZE         = 
DEFS             = -DF_CPU=$(F_CPU)UL
LIBS             =

#BURN_PROGRAMMER = avr109
#BURN_SERIALPORT = /dev/ttyUSB0
BURN_BAUDRATE   = 115200
BURN_PROGRAMMER = avrispmkII
BURN_SERIALPORT = usb


## Programs.
CC             = avr-gcc
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
SIZE           = avr-size

## Options common to compile, link and assembly rules.
COMMON = -mmcu=$(MCU_TARGET)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON) $(OPTIMIZE)
CFLAGS += -Wall -gdwarf-2 -std=gnu99 -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums $(DEFS)
CFLAGS += -I/usr/lib/avr/include  -I/home/ssafarik/src.hg/panels/C\ codes\ for\ Controller/Bootloader4Controller
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags.
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags.
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=$(TARGET).map
LDFLAGS += -Wl,-section-start=.text=0x20000



## Intel Hex file production flags.
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link.
OBJ = $(CSRC:.c=.o) $(ASRC:.S=.o)


## Build
all: $(TARGET).elf text $(TARGET).eep $(TARGET).lss size

## Compile
$(TARGET).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

%.o: %.S
	$(CC) -c $(ASMFLAGS) $< -o $@


%.eep: %.elf
	$(OBJCOPY) $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: %.elf
	$(OBJDUMP) -h -S $< > $@

size: ${TARGET}.hex
	@$(SIZE) -C --mcu=${MCU_TARGET} ${TARGET}.hex
    
    
text: hex
hex:  $(TARGET).hex

%.hex: %.elf
	$(OBJCOPY) -O ihex $(HEX_FLASH_FLAGS)  $< $@


## Clean target
.PHONY: clean install
clean:
	rm -rf $(OBJ) $(TARGET).elf dep/* $(TARGET).hex $(TARGET).eep $(TARGET).lss $(TARGET).map


## Write the bits onto the chip.
install:
	avrdude -p $(MCU_TARGET) -P $(BURN_SERIALPORT) -b $(BURN_BAUDRATE) -c $(BURN_PROGRAMMER) -e -U boot:w:$(TARGET).hex
	# Note: if you have trouble with avrdude writing to the boot section, try the following command (worked for me with
	# avrdude 6.01, compiled after libelf was installed).
	# avrdude -p $(MCU_TARGET) -P $(BURN_SERIALPORT) -b $(BURN_BAUDRATE) -c $(BURN_PROGRAMMER) -e -U boot:w:$(TARGET).elf


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

