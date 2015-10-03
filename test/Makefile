#      device
MCU=atmega1284p
MCU_FLASH=m1284p
F_CPU=16000000UL

#      file names
MAIN = test.c
RESULT = firmware

# sub source codes
SUB= \
	../avr-uart/uart.c \
	../fep.c \
	$(NULL)

CC   = /opt/local/bin/avr-gcc
COPY = /opt/local/bin/avr-objcopy
PROG = /opt/local/bin/avrdude

CFLAGS= -c -g -O0 -Wall -mmcu=$(MCU) -DF_CPU=$(F_CPU) -I.
CFLAGS+= -DUSART0_LARGE_BUFFER
CFLAGS+= -DUART_RX0_BUFFER_SIZE=256 -DUART_TX0_BUFFER_SIZE=512
# CFLAGS+= -DUSART1_ENABLED

.SUFFIXES: .c .o

.c.o:
	$(CC) $(CFLAGS) $< -o $(<:.c=.o)

#	result of Make is firmware.hex
all: $(RESULT).hex

#  create hex file from elf file
$(RESULT).hex: $(RESULT).elf
	$(COPY) -O ihex $(RESULT).elf $(RESULT).hex

#  create "executable and linkable format" file; link program.o
#  add other .o and .o dependencies here
$(RESULT).elf: $(MAIN:.c=.o) $(SUB:.c=.o)
	$(CC) $(MAIN:.c=.o) $(SUB:.c=.o) -Wl,-Map=$(MAIN:.c=).map,--cref -mmcu=$(MCU) -o $(RESULT).elf

#  make clean
clean:
	rm -rf *.o *.hex *.elf *.map *~ ../*.o ../avr-uart/*.o

#  make flash
flash:
	avrdude -c usbasp -p $(MCU_FLASH) -B 10 -U flash:w:$(RESULT).hex:i
flash-mk2:
	avrdude -c avrisp2 -p $(MCU_FLASH) -U flash:w:$(RESULT).hex:i
