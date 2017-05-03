# Makefile
# BCD -- Ben Hutton (blh36), Chris Leary (cdl28), Devrin Talen (dct23)
# ECE476 @ Cornell University


CC=avr-gcc
OPTIMIZE=-Os
DEFS=-D DEBUG_SIE -D DEBUG_HC
OBJCOPY=avr-objcopy
MCU_TARGET=atmega32
P_CFLAGS=-Wall $(OPTIMIZE) $(DEFS) -save-temps# optimize for size and warn all
override CFLAGS=-mmcu=$(MCU_TARGET) $(P_CFLAGS)# override needed by avr-lib build system
OC_HEX_FLAGS=-j .text -j .data -O ihex# avr object copy flags to make Intel hex
                                      # text and data segments extracted (EEPROM sold separately)
PROG_NAME=mega32_usb
OBJECTS=usb_primitives.o usb_interface.o usb_system.o usb_client.o
PROGRAMMER=avrdude
PROG_FLAGS=-v -c stk500v2 -P /dev/ttyUSB0 -p m32 -u -U# NOTE: this will sometimes be stk500v2!
PROG_HEX_PREFIX=flash:w:


##################
# ENTIRE PROGRAM #
##################


install:
	$(PROGRAMMER) $(PROG_FLAGS) $(PROG_HEX_PREFIX)$(PROG_NAME).hex


clean:
	-rm *.i *.s *.o *.elf *.hex


all: $(PROG_NAME).hex
	# all depends on the hex file


$(PROG_NAME).elf: $(OBJECTS)
	# binary depends on object consituents
	$(CC) $(CFLAGS) -o $@ $^


###################
# INFERENCE RULES #
###################


%.hex: %.elf
	# hex depends on the (elf) binary
	$(OBJCOPY) $(OC_HEX_FLAGS) $< $@


%.elf: %.c


%.o : %.c %.h
	# all of the objects depend on their corresponding C and H files
	$(CC) $(CFLAGS) -o $@ -c $<


#########
# SHELL #
#########


install_shell: shell
	# program the chip
	$(PROGRAMMER) $(PROG_FLAGS) $(PROG_HEX_PREFIX)test_shell_avr.hex


shell: $(OBJECTS) usb_shell.o
	# create the elf file
	$(CC) $(CFLAGS) -o test_shell_avr.elf $(OBJECTS) usb_shell.o test_shell_avr.c
	# make the hex
	$(OBJCOPY) $(OC_HEX_FLAGS) test_shell_avr.elf test_shell_avr.hex	


##########
# CLIENT #
##########


client: $(OBJECTS)
	# create the elf file
	$(CC) $(CFLAGS) -o test_client_avr.elf $(OBJECTS) test_client_avr.c
	# make the hex
	$(OBJCOPY) $(OC_HEX_FLAGS) test_client_avr.elf test_client_avr.hex


install_client: client
	# program the chip
	$(PROGRAMMER) $(PROG_FLAGS) $(PROG_HEX_PREFIX)test_client_avr.hex


test_client:
	# regression testing for the interface
	#gcc -o usb_primitives.o -DNO_AVR_GCC -c usb_primitives.c
	#gcc -o usb_interface.o -DNO_AVR_GCC -c usb_interface.o usb_interface.c
	#gcc -o test_system.o -DNO_AVR_GCC usb_system.c usb_interface.c usb_primitives.c test_system.c
	#gcc -o test_client.o -DNO_AVR_GCC usb_client.c usb_system.c usb_interface.c usb_primitives.c test_client.c
	./test_client.o
	rm test_client.o


##########
# SYSTEM #
##########


system: usb_primitives.o usb_interface.o usb_system.o
	# create the elf file
	$(CC) $(CFLAGS) -o test_system_avr.elf $(OBJECTS) test_system_avr.c
	# make the hex
	$(OBJCOPY) $(OC_HEX_FLAGS) test_system_avr.elf test_system_avr.hex

install_system: system
	# program the chip
	$(PROGRAMMER) $(PROG_FLAGS) $(PROG_HEX_PREFIX)test_system_avr.hex


test_system:
	# regression testing for the interface
	#gcc -o usb_primitives.o -DNO_AVR_GCC -c usb_primitives.c
	#gcc -o usb_interface.o -DNO_AVR_GCC -c usb_interface.o usb_interface.c
	gcc -o test_system.o -DNO_AVR_GCC usb_system.c usb_interface.c usb_primitives.c test_system.c
	./test_system.o
	rm test_system.o


#############
# INTERFACE #
#############


interface: usb_primitives.o usb_interface.o
	# create the elf file
	$(CC) $(CFLAGS) -o test_interface_avr.elf $(OBJECTS) test_interface_avr.c
	# make the hex
	$(OBJCOPY) $(OC_HEX_FLAGS) test_interface_avr.elf test_interface_avr.hex


install_interface: interface
	# program the chip
	$(PROGRAMMER) $(PROG_FLAGS) $(PROG_HEX_PREFIX)test_interface_avr.hex


test_interface:
	# regression testing for the interface
	#gcc -o usb_primitives.o -DNO_AVR_GCC -c usb_primitives.c
	#gcc -o usb_interface.o -DNO_AVR_GCC -c usb_interface.o usb_interface.c
	gcc -o test_interface.o -DNO_AVR_GCC -g usb_interface.c usb_primitives.c test_interface.c
	./test_interface.o
	rm test_interface.o


##############
# PRIMITIVES #
##############


primitives: usb_primitives.o
	# create the elf file
	$(CC) $(CFLAGS) -o test_primitives_avr.elf usb_primitives.o test_primitives_avr.c
	# make the hex
	$(OBJCOPY) $(OC_HEX_FLAGS) test_primitives_avr.elf test_primitives_avr.hex


install_primitives: primitives
	# program the chip
	$(PROGRAMMER) $(PROG_FLAGS) $(PROG_HEX_PREFIX)test_primitives_avr.hex


test_primitives:
	# regression testing for the primitives
	gcc -o usb_primitives.o -DNO_AVR_GCC -c usb_primitives.c
	gcc -o test_primitives.o -DNO_AVR_GCC usb_primitives.o test_primitives.c
	./test_primitives.o
	rm test_primitives.o


