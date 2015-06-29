mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
PROJECT := $(notdir $(patsubst %/,%,$(dir $(mkfile_path))))

LPROJECT  := lib$(PROJECT).a

SRC_PATH  := ./src
INC_PATH  := ./inc

INCLUDES  := -I./inc -I$(ROOT_PATH)/core/target/$(TARGET)/drivers/inc -I$(ROOT_PATH)/core/target/$(TARGET)/port/inc
#add external includes
INCLUDES  += -I$(ROOT_PATH)/external/tlsf/inc
INCLUDES  += $(addprefix -I, $(wildcard $(ROOT_PATH)/external/target/$(TARGET)/*/inc))

SRC_FILES := $(wildcard $(SRC_PATH)/*.c)
INC_FILES := $(wildcard $(INC_PATH)/*.h)
OBJ_FILES := $(subst $(SRC_PATH),$(OBJ_PATH),$(SRC_FILES:.c=.o))

DRIVERS_SRC_PATH := ../target/$(TARGET)/drivers/src
DRIVERS_SRC_FILES := $(wildcard $(DRIVERS_SRC_PATH)/*.c)
DRIVERS_OBJ_FILES := $(subst $(DRIVERS_SRC_PATH),$(OBJ_PATH),$(DRIVERS_SRC_FILES:.c=.o))

PORT_SRC_PATH := ../target/$(TARGET)/port/src
PORT_SRC_FILES := $(wildcard $(PORT_SRC_PATH)/*.c)
PORT_OBJ_FILES := $(subst $(PORT_SRC_PATH),$(OBJ_PATH),$(PORT_SRC_FILES:.c=.o))

$(LPROJECT):  $(PORT_OBJ_FILES) $(DRIVERS_OBJ_FILES) $(OBJ_FILES)
	@echo "*** Archiving project $(PROJECT) ***"
	arm-none-eabi-ar -r $(OUT_PATH)/$(LPROJECT) $(OBJ_FILES)
	arm-none-eabi-size $(OUT_PATH)/$(LPROJECT)
	@echo ""

$(OBJ_PATH)/%.o: $(PORT_SRC_PATH)/%.c
	@echo "*** Compiling C file $< ***"
	arm-none-eabi-gcc $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< -o $@
	@echo ""

$(OBJ_PATH)/%.o: $(DRIVERS_SRC_PATH)/%.c
	@echo "*** Compiling C file $< ***"
	arm-none-eabi-gcc $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< -o $@
	@echo ""

$(OBJ_PATH)/%.o: $(SRC_PATH)/%.c
	@echo "*** Compiling C file $< ***"
	arm-none-eabi-gcc $(SYMBOLS) $(INCLUDES) $(CFLAGS) $< -o $@
	@echo ""
