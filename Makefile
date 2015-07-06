export PROJECT		  := blinky
export PROJECT_PATH := examples/$(PROJECT)
export TARGET       := lpc4337

#export ROOT_PATH := $(shell pwd)
export ROOT_PATH := .
export OUT_PATH  := out
export OBJ_PATH  := $(OUT_PATH)/obj

ifeq ($(TARGET),lpc4337)
export CFLAGS    := -Wall -ggdb3 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c
export SYMBOLS   := -DDEBUG -DCORE_M4 -D__USE_LPCOPEN -D__LPC43XX__ -D__CODE_RED -D__USE_UPOSIX_RTOS
export LFLAGS    := -nostdlib -fno-builtin -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
endif

TARGET_LIB := $(wildcard external/target/$(TARGET)/*)

SRC_PATH := external/tlsf/src
SRC_PATH += $(foreach dir, $(TARGET_LIB), $(dir)/src)
SRC_PATH += core/target/$(TARGET)/drivers/src
SRC_PATH += core/target/$(TARGET)/port/src
SRC_PATH += core/uposix/src
SRC_PATH += $(PROJECT_PATH)/src

C_FILES := $(foreach dir, $(SRC_PATH), $(wildcard $(dir)/*.c))
ASM_FILES := $(foreach dir, $(SRC_PATH), $(wildcard $(dir)/*.S))

SRC_FILES := $(C_FILES) $(ASM_FILES)

INC_PATH := external/tlsf/inc
INC_PATH += $(foreach dir, $(TARGET_LIB), $(dir)/inc)
INC_PATH += core/target/$(TARGET)/drivers/inc
INC_PATH += core/target/$(TARGET)/port/inc
INC_PATH += core/uposix/inc
INC_PATH += $(PROJECT_PATH)/inc

INCLUDES := $(addprefix -I,$(INC_PATH))

vpath %.c $(SRC_PATH)
vpath %.S $(SRC_PATH)
vpath %.s $(SRC_PATH)

OBJ_FILES := $(addprefix $(OBJ_PATH)/,$(notdir $(C_FILES:.c=.o)))
OBJ_FILES += $(addprefix $(OBJ_PATH)/,$(notdir $(ASM_FILES:.S=.o)))

MAP_FILE := -Xlinker -Map=$(OUT_PATH)/map.map

LD_FILE  := -Tconfig/$(TARGET)/ld/$(TARGET)_lib.ld
LD_FILE  += -Tconfig/$(TARGET)/ld/$(TARGET)_mem.ld
LD_FILE  += -Tconfig/$(TARGET)/ld/$(TARGET).ld

%.o: %.c
	@echo "*** Compiling C file $< ***"
	arm-none-eabi-gcc $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< -o $(OBJ_PATH)/$@
	@echo ""

%.o: %.S
	@echo "*** Compiling Assembly file $< ***"
	arm-none-eabi-gcc $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< -o $(OBJ_PATH)/$@
	@echo ""

all: $(PROJECT)

$(PROJECT): $(notdir $(OBJ_FILES))
	@echo "*** Linking $@ ***"
	arm-none-eabi-gcc $(LIB_PATH) $(LFLAGS) $(MAP_FILE) $(LD_FILE) -o $(OUT_PATH)/$(PROJECT).axf $(OBJ_FILES) $(LIBS)
	arm-none-eabi-size $(OUT_PATH)/$(PROJECT).axf
	arm-none-eabi-objcopy -v -O binary $(OUT_PATH)/$(PROJECT).axf $(OUT_PATH)/$(PROJECT).bin
	@echo ""

clean:
	rm -f $(OBJ_PATH)/*.*
	rm -f $(OUT_PATH)/*.*

info:
	@echo $(SRC_PATH)
	@echo ""
	@echo $(SRC_FILES)
	@echo ""
	@echo $(OBJ_FILES)
