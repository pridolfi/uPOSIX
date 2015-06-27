export TARGET    := lpc4337

export ROOT_PATH := $(shell pwd)

export OUT_PATH  := $(ROOT_PATH)/out
export OBJ_PATH  := $(OUT_PATH)/obj

ifeq ($(TARGET),lpc4337)
export CFLAGS    := -Wall -ggdb3 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c
export SYMBOLS   := -DDEBUG -DCORE_M4 -D__USE_LPCOPEN -D__LPC43XX__ -D__CODE_RED
endif

all:
	make -C ./external/tlsf

clean:
	rm -f $(OBJ_PATH)/*.*
	rm -f $(OUT_PATH)/*.*
	
