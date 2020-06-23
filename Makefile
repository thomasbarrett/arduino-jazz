PORT=/dev/cu.usbmodem14101
BLUETOOTH_PORT=/dev/cu.Arduino-DevB
MCU=atmega328p
CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) -Os -DF_CPU=16000000UL
LDFLAGS=-Wl,-gc-sections -Wl,-relax
CC=avr-gcc
TARGET=main
OBJECT_FILES=main.c

all: $(TARGET).hex

clean:
	rm -f *.o *.hex *.obj *.hex

%.hex: %.obj
	avr-objcopy -R .eeprom -O ihex $< $@

%.obj: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

deploy: $(TARGET).hex
	avrdude -p $(MCU) -c arduino -P $(PORT) -U flash:w:$(TARGET).hex

listen:
	socat stdio $(BLUETOOTH_PORT) 