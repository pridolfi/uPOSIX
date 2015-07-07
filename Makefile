PROJECT		 := examples/blinky
TARGET       := lpc1769

PROJECT_NAME := $(notdir $(PROJECT))
ROOT_PATH := .
OUT_PATH  := out
OBJ_PATH  := $(OUT_PATH)/obj

ifeq ($(TARGET),lpc4337)
CFLAGS  := -Wall -ggdb3 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -c
SYMBOLS := -DDEBUG -DCORE_M4 -D__USE_LPCOPEN -D__LPC43XX__ -D__CODE_RED -D__USE_UPOSIX_RTOS
LFLAGS  := -nostdlib -fno-builtin -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
else
ifeq ($(TARGET),lpc1769)
CFLAGS  := -Wall -ggdb3 -mcpu=cortex-m3 -mthumb -c
SYMBOLS := -DDEBUG -DCORE_M3 -D__USE_LPCOPEN -D__LPC17XX__ -D__CODE_RED -D__USE_UPOSIX_RTOS
LFLAGS  := -nostdlib -fno-builtin -mcpu=cortex-m3 -mthumb
else
$(error Please define TARGET variable!)
endif
endif

EXT_PATH := $(wildcard external/target/$(TARGET)/*)

SRC_PATH := external/tlsf/src
SRC_PATH += $(foreach dir, $(EXT_PATH), $(dir)/src)
SRC_PATH += core/target/$(TARGET)/drivers/src
SRC_PATH += core/target/$(TARGET)/port/src
SRC_PATH += core/uposix/src
SRC_PATH += $(PROJECT)/src

C_FILES := $(foreach dir, $(SRC_PATH), $(wildcard $(dir)/*.c))
ASM_FILES := $(foreach dir, $(SRC_PATH), $(wildcard $(dir)/*.S))

SRC_FILES := $(C_FILES) $(ASM_FILES)

INC_PATH := external/tlsf/inc
INC_PATH += $(foreach dir, $(EXT_PATH), $(dir)/inc)
INC_PATH += core/target/$(TARGET)/drivers/inc
INC_PATH += core/target/$(TARGET)/port/inc
INC_PATH += core/uposix/inc
INC_PATH += $(PROJECT)/inc

INCLUDES := $(addprefix -I,$(INC_PATH))

vpath %.c $(SRC_PATH)
vpath %.S $(SRC_PATH)
vpath %.s $(SRC_PATH)
vpath %.o $(OBJ_PATH)

OBJ_FILES := $(addprefix $(OBJ_PATH)/,$(notdir $(C_FILES:.c=.o)))
OBJ_FILES += $(addprefix $(OBJ_PATH)/,$(notdir $(ASM_FILES:.S=.o)))

MAP_FILE := -Xlinker -Map=$(OUT_PATH)/$(PROJECT_NAME).map

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

all: $(PROJECT_NAME)

$(PROJECT_NAME): $(notdir $(OBJ_FILES))
	@echo "*** Linking $@ ***"
	arm-none-eabi-gcc $(LIB_PATH) $(LFLAGS) $(MAP_FILE) $(LD_FILE) -o $(OUT_PATH)/$(PROJECT_NAME).axf $(OBJ_FILES) $(LIBS)
	arm-none-eabi-size $(OUT_PATH)/$(PROJECT_NAME).axf
	arm-none-eabi-objcopy -v -O binary $(OUT_PATH)/$(PROJECT_NAME).axf $(OUT_PATH)/$(PROJECT_NAME).bin
	@echo ""

clean:
	rm -f $(OBJ_PATH)/*.*
	rm -f $(OUT_PATH)/*.*

info:
	@echo PROJECT_NAME: $(PROJECT_NAME)
	@echo ""
	@echo SRC_PATH:  $(SRC_PATH)
	@echo ""
	@echo SRC_FILES: $(SRC_FILES)
	@echo ""
	@echo OBJ_FILES: $(OBJ_FILES)
