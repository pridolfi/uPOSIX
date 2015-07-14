# Copyright 2015, Pablo Ridolfi
#
# This file is part of uPOSIX.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Include Makefile.mine with PROJECT and TARGET definitions.
-include Makefile.mine

# Project location, relative to uPOSIX path (default, to be defined in
# Makefile.mine).
PROJECT ?= examples/blinky

# Target processor (default, to be defined in Makefile.mine):
# +---------+-----------------------------------------------------------------+
# | TARGET  |  Description                                                    |
# +---------+-----------------------------------------------------------------+
# | lpc1769 |  NXP LPC1769 Cortex-M3 microcontroller.                         |
# +---------+-----------------------------------------------------------------+
# | lpc4337 |  NXP LPC4337 Cortex-M4/M0 dual-core microcontroller.            |
# +---------+-----------------------------------------------------------------+
TARGET ?= lpc1769

# Internal variables, do not modify! Or modify carefully :-)
PROJECT_NAME := $(notdir $(PROJECT))
ROOT_PATH := .
OUT_PATH  := out
OBJ_PATH  := $(OUT_PATH)/obj

# Compiler flags, defined Symbols and Linker flags, target dependent
ifeq ($(TARGET),lpc4337)
CFLAGS  := -Wall -ggdb3 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -fdata-sections -ffunction-sections -c
SYMBOLS := -DDEBUG -DCORE_M4 -D__USE_LPCOPEN -D__LPC43XX__ -D__CODE_RED -D__USE_UPOSIX_RTOS
LFLAGS  := -nostdlib -fno-builtin -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -Wl,--gc-sections
else
ifeq ($(TARGET),lpc1769)
CFLAGS  := -Wall -ggdb3 -mcpu=cortex-m3 -mthumb -fdata-sections -ffunction-sections -c
SYMBOLS := -DDEBUG -DCORE_M3 -D__USE_LPCOPEN -D__LPC17XX__ -D__CODE_RED -D__USE_UPOSIX_RTOS
LFLAGS  := -nostdlib -fno-builtin -mcpu=cortex-m3 -mthumb -Wl,--gc-sections
else
$(error Please define TARGET variable!)
endif
endif

# External library paths
EXT_PATH := $(wildcard external/target/$(TARGET)/*)

# Source files paths
SRC_PATH := external/tlsf/src
SRC_PATH += $(foreach dir, $(EXT_PATH), $(dir)/src)
SRC_PATH += core/target/$(TARGET)/drivers/src
SRC_PATH += core/target/$(TARGET)/port/src
SRC_PATH += core/uposix/src
SRC_PATH += $(PROJECT)/src

# Include files paths
INC_PATH := external/tlsf/inc
INC_PATH += $(foreach dir, $(EXT_PATH), $(dir)/inc)
INC_PATH += core/target/$(TARGET)/drivers/inc
INC_PATH += core/target/$(TARGET)/port/inc
INC_PATH += core/uposix/inc
INC_PATH += $(PROJECT)/inc

# Add '-I' option to each include path for GCC
INCLUDES := $(addprefix -I,$(INC_PATH))

# Add C source files paths to VPATH
vpath %.c $(SRC_PATH)

# Add assembly source files paths to VPATH
vpath %.S $(SRC_PATH)

# Add object files path to VPATH
vpath %.o $(OBJ_PATH)

# Get source file list with path
C_FILES   := $(foreach dir, $(SRC_PATH), $(wildcard $(dir)/*.c))
ASM_FILES := $(foreach dir, $(SRC_PATH), $(wildcard $(dir)/*.S))

# Get object file list with relative path
OBJ_FILES := $(addprefix $(OBJ_PATH)/,$(notdir $(C_FILES:.c=.o)))
OBJ_FILES += $(addprefix $(OBJ_PATH)/,$(notdir $(ASM_FILES:.S=.o)))

# Get object file list without path (for dependencies)
OBJS := $(notdir $(OBJ_FILES))

# Map file output for link step
MAP_FILE := -Xlinker -Map=$(OUT_PATH)/$(PROJECT_NAME).map

# Linker script files
LD_FILE  := -Tconfig/$(TARGET)/ld/$(TARGET)_lib.ld
LD_FILE  += -Tconfig/$(TARGET)/ld/$(TARGET)_mem.ld
LD_FILE  += -Tconfig/$(TARGET)/ld/$(TARGET).ld

# Compilation rule for C files
%.o: %.c
	@echo "*** Compiling C file $< ***"
	arm-none-eabi-gcc $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< -o $(OBJ_PATH)/$@
	@echo ""

# Compilation rule for assembly files
%.o: %.S
	@echo "*** Compiling Assembly file $< ***"
	arm-none-eabi-gcc $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< -o $(OBJ_PATH)/$@
	@echo ""

# All rule: make project
all: $(PROJECT_NAME)

# Link rule
$(PROJECT_NAME): $(OBJS)
	@echo "*** Linking $@ ***"
	arm-none-eabi-gcc $(LIB_PATH) $(LFLAGS) $(MAP_FILE) $(LD_FILE) -o $(OUT_PATH)/$(PROJECT_NAME).axf $(OBJ_FILES) $(LIBS)
	arm-none-eabi-size $(OUT_PATH)/$(PROJECT_NAME).axf
	arm-none-eabi-objcopy -v -O binary $(OUT_PATH)/$(PROJECT_NAME).axf $(OUT_PATH)/$(PROJECT_NAME).bin
	@echo ""

# Clean rule (removes objects, binaries and other generated files)
clean:
	rm -f $(OBJ_PATH)/*.*
	rm -f $(OUT_PATH)/*.*

# Download rule using openocd
download:
	@echo "*** Downloading $(OUT_PATH)/$(PROJECT_NAME).bin to target $(TARGET) ***"
	openocd -f config/$(TARGET)/openocd/$(TARGET).cfg  -c "init" -c "halt 0" -c "flash write_image erase unlock $(OUT_PATH)/$(PROJECT_NAME).bin 0x1A000000 bin" -c "reset run" -c "exit"

# Info rule for displaying some variables
info:
	@echo PROJECT_NAME: $(PROJECT_NAME)
	@echo ""
	@echo SRC_PATH:  $(SRC_PATH)
	@echo ""
	@echo SRC_FILES: $(SRC_FILES)
	@echo ""
	@echo OBJ_FILES: $(OBJ_FILES)
	@echo ""
	@echo MAKECMDGOALS: $(MAKECMDGOALS)

###############################################################################
# END OF MAKEFILE                                                             #
###############################################################################
