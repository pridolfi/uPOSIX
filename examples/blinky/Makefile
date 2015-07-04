mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
PROJECT := $(notdir $(patsubst %/,%,$(dir $(mkfile_path))))

SRC_PATH := ./src
INC_PATH := ./inc

#application include path
INCLUDES := -I./inc

#add uposix and external includes
INCLUDES  += -I$(ROOT_PATH)/core/uposix/inc -I$(ROOT_PATH)/core/target/$(TARGET)/drivers/inc -I$(ROOT_PATH)/core/target/$(TARGET)/port/inc
INCLUDES  += -I$(ROOT_PATH)/external/tlsf/inc
INCLUDES  += $(addprefix -I, $(wildcard $(ROOT_PATH)/external/target/$(TARGET)/*/inc))

LIB_PATH := -L$(OUT_PATH)
LIBS := -llpc_chip_lpc43xx -llpc_board_ngx_xplorer_4330 -lboot_lpc4337 -ltlsf -luposix

LD_FILE  := -T$(ROOT_PATH)/external/target/$(TARGET)/ld/$(TARGET)_lib.ld
LD_FILE  += -T$(ROOT_PATH)/external/target/$(TARGET)/ld/$(TARGET)_mem.ld
LD_FILE  += -T$(ROOT_PATH)/external/target/$(TARGET)/ld/$(TARGET).ld

SRC_FILES := $(wildcard $(SRC_PATH)/*.c)
ASM_FILES := $(wildcard $(SRC_PATH)/*.S)
INC_FILES := $(wildcard $(INC_PATH)/*.h)
OBJ_FILES := $(subst $(SRC_PATH),$(OBJ_PATH),$(SRC_FILES:.c=.o))
OBJ_FILES += $(subst $(SRC_PATH),$(OBJ_PATH),$(ASM_FILES:.S=.o))

MAP_FILE := -Xlinker -Map=$(OUT_PATH)/$(PROJECT).map

$(PROJECT): $(OBJ_FILES)
	@echo "*** Linking project $(PROJECT) ***"
	arm-none-eabi-gcc $(LIB_PATH) $(LFLAGS) $(MAP_FILE) $(LD_FILE) -o $(OUT_PATH)/$(PROJECT).axf $(OBJ_FILES) $(LIBS)
	arm-none-eabi-size $(OUT_PATH)/$(PROJECT).axf
	arm-none-eabi-objcopy -v -O binary $(OUT_PATH)/$(PROJECT).axf $(OUT_PATH)/$(PROJECT).bin
	@echo ""

-include $(wildcard $(OBJ_PATH)/*.d)

$(OBJ_PATH)/%.o: $(SRC_PATH)/%.c
	@echo "*** Compiling C file $< ***"
	arm-none-eabi-gcc $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< -o $@
	arm-none-eabi-gcc -MM $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< > $(OBJ_PATH)/$*.d
	@echo ""

$(OBJ_PATH)/%.o: $(SRC_PATH)/%.S
	@echo "*** Compiling Assembly file $< ***"
	arm-none-eabi-gcc $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< -o $@
	arm-none-eabi-gcc -MM $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< > $(OBJ_PATH)/$*.d
	@echo ""
