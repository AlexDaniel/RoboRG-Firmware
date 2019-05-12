DEVICE          = STM32F103C8T6
OPENCM3_DIR     = libopencm3
OBJS            += src/main.o src/timing.o src/lanc.o src/cdcacm.o src/input.o src/motors.o

# TODO use -Wpedantic
CFLAGS          += -O3 -ggdb3
CPPFLAGS        += -MD -Wall
CXXFLAGS        += -std=c++17
LDFLAGS         += -static -nostartfiles
LDLIBS          += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

include $(OPENCM3_DIR)/mk/genlink-config.mk
include $(OPENCM3_DIR)/mk/gcc-config.mk

.PHONY: clean all

all: binary.elf binary.bin

flash: all
	st-flash write binary.bin 0x8000000

clean:
	$(Q)$(RM) -rf binary.* src/*.d

include $(OPENCM3_DIR)/mk/genlink-rules.mk
include $(OPENCM3_DIR)/mk/gcc-rules.mk
