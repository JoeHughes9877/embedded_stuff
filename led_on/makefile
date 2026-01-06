# MCU type
MCU = atmega328p

# CPU frequency
F_CPU = 16000000UL

# Compiler and tools
CC = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude
PORT = /dev/ttyACM0
BAUD = 115200

# Source and output
SRC = main.c
ELF = main.elf
HEX = main.hex

# Compiler flags
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall

# Default: build everything
all: $(HEX)

# Compile C into ELF
$(ELF): $(SRC)
	$(CC) $(CFLAGS) -o $@ $<

# Convert ELF to HEX
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

# Flash the HEX to the board
flash: $(HEX)
	$(AVRDUDE) -c arduino -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:$<

# Clean build files
clean:
	rm -f $(ELF) $(HEX)
