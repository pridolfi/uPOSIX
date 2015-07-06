export APPLICATION := examples/blinky
export TARGET      := lpc4337

#export ROOT_PATH := $(shell pwd)
export ROOT_PATH := .
export OUT_PATH  := $(ROOT_PATH)/out
export OBJ_PATH  := $(OUT_PATH)/obj

ifeq ($(TARGET),lpc4337)
export CFLAGS    := -Wall -ggdb3 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c
export SYMBOLS   := -DDEBUG -DCORE_M4 -D__USE_LPCOPEN -D__LPC43XX__ -D__CODE_RED -D__USE_UPOSIX_RTOS -D__REDLIB__
export LFLAGS    := -nostdlib -fno-builtin -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
endif

TARGET_LIB := $(wildcard $(ROOT_PATH)/external/target/$(TARGET)/*)

SRC_FILES := $(wildcard $(ROOT_PATH)/external/tlsf/src/*.c)
SRC_FILES += $(foreach dir, $(TARGET_LIB), $(wildcard $(dir)/src/*.c))
SRC_FILES += $(wildcard $(ROOT_PATH)/core/target/$(TARGET)/drivers/src/*.c)
SRC_FILES += $(wildcard $(ROOT_PATH)/core/target/$(TARGET)/port/src/*.c)
SRC_FILES += $(wildcard $(ROOT_PATH)/core/uposix/src/*.c)

OBJ_FILES := $(addprefix $(OBJ_PATH)/,$(notdir $(SRC_FILES:.c=.o)))

all: $(notdir $(OBJ_FILES))
	@echo $(SRC_FILES)
	@echo ""
	@echo $(OBJ_FILES)

$(OBJ_PATH)/%.o: %.c
	@echo "*** Compiling C file $< ***"
	@echo ""

clean:
	rm -f $(OBJ_PATH)/*.*
	rm -f $(OUT_PATH)/*.*
