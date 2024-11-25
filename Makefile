HACKRFSDR ?= no
PLUTOSDR ?= no
DEBUG ?= no
# Detect the operating system
UNAME_S := $(shell uname -s)

DIALECT = -std=c11
CFLAGS += $(DIALECT) -W -Wall -D_GNU_SOURCE
LIBS = -lm -pthread -lpthread -lcurl -lz -lpanel -lncurses
LDFLAGS =
SDR_OBJ = sdr_iqfile.o

ifeq ($(DEBUG), yes)
    CFLAGS += -Og -g
endif

# Compilation on Mac OS
ifeq ($(UNAME_S),Darwin)
    LIBS += -largp
    LDFLAGS = -L/opt/homebrew/lib
    CFLAGS += -I/opt/homebrew/include
endif

ifeq ($(HACKRFSDR), yes)
    SDR_OBJ += sdr_hackrf.o
    CPPFLAGS += -DENABLE_HACKRFSDR
    CFLAGS += $(shell pkg-config --cflags libhackrf)
    LIBS_SDR += $(shell pkg-config --libs libhackrf)
endif

ifeq ($(PLUTOSDR), yes)
    SDR_OBJ += sdr_pluto.o
    CPPFLAGS += -DENABLE_PLUTOSDR
    CFLAGS += $(shell pkg-config --cflags libiio libad9361)
    LIBS_SDR += $(shell pkg-config --libs libiio libad9361)
endif

all: gps-sim

%.o: %.c *.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

gps-sim: fifo.o almanac.o gps.o gui.o sdr.o gps-sim.o $(SDR_OBJ) $(COMPAT)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(LIBS_SDR)

clean:
	rm -f *.o  gps-sim
