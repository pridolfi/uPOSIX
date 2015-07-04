export APPLICATION := examples/blinky
export TARGET      := lpc4337

export ROOT_PATH := $(shell pwd)
export OUT_PATH  := $(ROOT_PATH)/out
export OBJ_PATH  := $(OUT_PATH)/obj

ifeq ($(TARGET),lpc4337)
export CFLAGS    := -Wall -ggdb3 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c
export SYMBOLS   := -DDEBUG -DCORE_M4 -D__USE_LPCOPEN -D__LPC43XX__ -D__CODE_RED -D__USE_UPOSIX_RTOS -D__REDLIB__
export LFLAGS    := -nostdlib -fno-builtin -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
endif

TARGET_LIB := $(wildcard $(ROOT_PATH)/external/target/$(TARGET)/*)

all:
	@echo "*** Building TLSF library ***"
	make -C ./external/tlsf

	@for PROJECT in $(TARGET_LIB) ; do \
			echo "*** Building external target library $$PROJECT ***" ; \
			make -C $$PROJECT ; \
			echo "" ; \
	done

	@echo "*** Building main uPOSIX library ***"
	make -C ./core

	@echo "*** Building and linking application ***"
	make -C ./$(APPLICATION)

clean:
	rm -f $(OBJ_PATH)/*.*
	rm -f $(OUT_PATH)/*.*
