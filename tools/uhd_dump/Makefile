INCLUDES = usrp3_regs.h uhd_dump.h

BINARIES = vrlp_log

OBJECTS = uhd_dump.o 

CFLAGS = -g -O0 -Wall

LDFLAGS = -lpcap -lm -L/usr/lib/x86_64-linux-gnu -static /usr/lib/x86_64-linux-gnu/libpcap.a

CC = cc

.PHONY: all
all: $(BINARIES)

vrlp_log: uhd_dump.o  vrlp_log.o $(INCLUDES)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@  uhd_dump.o  vrlp_log.o



clean:
	rm $(BINARIES)  $(BINARIES:%=%.o) $(OBJECTS)

