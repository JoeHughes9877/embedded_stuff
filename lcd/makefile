# MCU type
MCU = atmega328p

# CPU frequency
F_CPU = 16000000UL

# Tools
CC = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

# Upload settings
PORT = /dev/ttyUSB0
BAUD = 115200
PROGRAMMER = arduino

# Files
SRC = lcd.c
ELF = lcd.elf
HEX = lcd.hex

# Flags
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall
LDFLAGS = -mmcu=$(MCU)

# Default target
all: $(HEX)

# Compile
$(ELF): $(SRC)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

# Convert to HEX
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

# Flash
flash: $(HEX)
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b $(BAUD) \
	-U flash:w:$<:i

# Clean
clean:
	rm -f $(ELF) $(HEX)
