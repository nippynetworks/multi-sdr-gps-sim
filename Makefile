HACKRFSDR ?= no
PLUTOSDR ?= no
DEBUG ?= no
OPENMP ?= no
# Detect the operating system
UNAME_S := $(shell uname -s)

DIALECT = -std=c11
CFLAGS += $(DIALECT) -W -Wall -D_GNU_SOURCE
LIBS = -lm -pthread -lpthread -lcurl -lz -lpanel -lncurses
LDFLAGS =
SDR_OBJ = sdr_iqfile.o

ifeq ($(DEBUG), yes)
    CFLAGS += -O0 -g
else
    CFLAGS += -O3 -ftree-vectorize -funroll-loops -flto=auto
    CFLAGS += -march=native
    #CFLAGS += -fopt-info-vec-missed -fopt-info-vec
    # CFLAGS += -Rpass=vectorize -Rpass-analysis=vectorize

    # Fast math
    CFLAGS +=-ffast-math
    LDFLAGS += -ffast-math
endif

ifeq ($(OPENMP), yes)
    # Parallelization
    ifeq ($(UNAME_S),Darwin)
        CFLAGS += -DOPENMP -I/opt/homebrew/opt/libomp/include -Xpreprocessor -fopenmp
        LDFLAGS += -L/opt/homebrew/opt/libomp/lib -lomp
    else
        CFLAGS += -DOPENMP -fopenmp
        LDFLAGS += -fopenmp
    endif
endif

# Compilation on Mac OS
ifeq ($(UNAME_S),Darwin)
    LIBS += -largp
    LDFLAGS += -L/opt/homebrew/lib
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

gps-sim: fifo.o almanac.o download.o ephemeris.o gps.o gui.o sdr.o time.o gps-sim.o $(SDR_OBJ) $(COMPAT)
	$(CC) -g -o $@ $^ $(LDFLAGS) $(LIBS) $(LIBS_SDR)

clean:
	rm -f *.o  gps-sim
