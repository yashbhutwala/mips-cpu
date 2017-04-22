# Makefile to generate bare metal code to run on a simulated (Verilog) processor
# Bucknell University
# Alan Marchiori 2014
CC=mipsel-linux-gcc
AS=mipsel-linux-as
LD=mipsel-linux-ld

SREC=srec_cat

# these are the flags we need for bare metal code generation
CFLAGS=-mno-abicalls -fpic -nostdlib -static
LDFLAGS=-L/usr/remote/mipsel/lib/gcc/mipsel-buildroot-linux-uclibc/4.6.3 -lgcc
ASMSOURCE=puts.s start.s
CSOURCE=fib.c memset.c itoa.c

ASMOBJ=$(ASMSOURCE:.s=.o)
COBJ=$(CSOURCE:.c=.o)
OBJECTS=$(ASMOBJ) $(COBJ)

BIN_OUTPUT=fib
GCC_OUTPUT=fib.s
VERILOG_OUTPUT=fib.v

mips: mips.v mips_tb.v
	iverilog mips.v mips_tb.v -o mips

all: $(OBJECTS) mips
	# finally build an executable
	# link twice, once to generate an ELF MIPS executable 
	# you can run mipsel-linux-objdump on this to inspect
	$(LD) $(LDFLAGS) $(OBJECTS) -o $(BIN_OUTPUT)

	# now link to a motorola SRecord
	$(LD) $(LDFLAGS) --oformat=srec $(OBJECTS) -o $(GCC_OUTPUT)
	# convert the SRecord file into a Verilog file
	$(SREC) $(GCC_OUTPUT) -Byte-swap 4 -o $(VERILOG_OUTPUT) -VMem

%.o: %.c
	# compile C to object files as usual
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	# assemble to a motorola srecord file
	$(AS) $< -o $@

clean:
	rm -f $(OUTPUT) $(OBJECTS) mips a.out $(BIN_OUTPUT) $(GCC_OUTPUT) $(VERILOG_OUTPUT)
