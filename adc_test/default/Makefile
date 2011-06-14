###############################################################################
# Makefile for the project pushpak
###############################################################################

## General Flags
PROJECT = pushpak
MCU = atmega644p
TARGET = pushpak.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99                -ffunction-sections  -fdata-sections       -DF_CPU=20000000UL -O0 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=pushpak.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Objects that must be built in order to link
OBJECTS = Print.o HardwareSerial.o cpp_hack.o adc.o adc_test.o messages.o accelerometer.o gyrosensor.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) pushpak.hex pushpak.eep pushpak.lss size

## Compile
Print.o: ../Print.C
	$(CC) $(INCLUDES) $(CFLAGS) -c -fno-exceptions $<

HardwareSerial.o: ../HardwareSerial.C
	$(CC) $(INCLUDES) $(CFLAGS) -c -fno-exceptions $<

cpp_hack.o: ../cpp_hack.C
	$(CC) $(INCLUDES) $(CFLAGS) -c -fno-exceptions $<

adc.o: ../adc.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

adc_test.o: ../adc_test.C
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

messages.o: ../messages.C
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

accelerometer.o: ../accelerometer.C
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

gyrosensor.o: ../gyrosensor.C
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) pushpak.elf dep/* pushpak.hex pushpak.eep pushpak.lss pushpak.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

