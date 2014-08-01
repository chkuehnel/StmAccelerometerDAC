ROOT=$(shell pwd)
PROJ_NAME = stm32f3

###################################################
# Set toolchain
#TC = /opt/ARM/arm-linaro-eabi-4.6/bin/arm-none-eabi
#TC = ~/CodeSourcery/bin/arm-none-eabi
#TC = ~/CodeSourcery/Sourcery_CodeBench_Lite_for_ARM_EABI/bin/arm-none-eabi
TC = /home/christian/Downloads/gcc-arm-none-eabi-4_8-2014q2/bin/arm-none-eabi
#TC = C:\Program Files\Atollic\TrueSTUDIO for ARM Lite 3.2.0\ARMTools\bin\arm-atollic-eabi
#TC = C:\yagarto\bin\arm-none-eabi
#TC = C:\Program Files\yagarto4.6.2\bin\arm-none-eabi

# Set Tools
CC			= $(TC)-gcc
AR			= $(TC)-ar
OBJCOPY		= $(TC)-objcopy
OBJDUMP		= $(TC)-objdump
SIZE		= $(TC)-size

###################################################
# Set Sources
LIB_SRCS	= $(wildcard Libraries/STM32F30x_StdPeriph_Driver/src/*.c)
USER_SRCS	= $(wildcard src/*.c)

# Set Objects
LIB_OBJS	= $(LIB_SRCS:.c=.o)
USER_OBJS	= $(USER_SRCS:.c=.o) src/startup_stm32f30x.o

# Set Include Paths
INCLUDES 	= -ILibraries/STM32F30x_StdPeriph_Driver/inc/ \
			-ILibraries/CMSIS/Include \
			-ILibraries/CMSIS/Device/ST/STM32F30x/Include \
			-Iinc/
			
# Set Libraries
LIBS		= -lm -lc

###################################################
# Set Board
MCU 		= -mthumb -mcpu=cortex-m4
FPU 		= -mfpu=fpv4-sp-d16 -mfloat-abi=hard
#FPU 		= -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
DEFINES 	= -DSTM32F3XX -DUSE_STDPERIPH_DRIVER

# Set Compilation and Linking Flags
CFLAGS 		= $(MCU) $(FPU) $(DEFINES) $(INCLUDES) \
			-g -Wall -std=gnu90 -O0 -ffunction-sections -fdata-sections
ASFLAGS 	= $(MCU) $(FPU) -g -Wa,--warn -x assembler-with-cpp
LDFLAGS 	= $(MCU) $(FPU) -g -gdwarf-2\
			-Tstm32f30_flash.ld \
			-Xlinker --gc-sections -Wl,-Map=$(PROJ_NAME).map \
			$(LIBS) \
			-o $(PROJ_NAME).elf

###################################################
# Default Target
all: $(PROJ_NAME).bin info

# elf Target
$(PROJ_NAME).elf: $(LIB_OBJS) $(USER_OBJS)
	@$(CC) $(LIB_OBJS) $(USER_OBJS) $(LDFLAGS)
	@echo $@

# bin Target
$(PROJ_NAME).bin: $(PROJ_NAME).elf
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	@echo $@

#$(PROJ_NAME).hex: $(PROJ_NAME).elf
#	@$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
#	@echo $@

#$(PROJ_NAME).lst: $(PROJ_NAME).elf
#	@$(OBJDUMP) -h -S $(PROJ_NAME).elf > $(PROJ_NAME).lst
#	@echo $@

# Display Memory Usage Info
info: $(PROJ_NAME).elf
	@$(SIZE) --format=berkeley $(PROJ_NAME).elf

# Rule for .c files
.c.o:
	@$(CC) $(CFLAGS) -c -o $@ $<
	@echo $@

# Rule for .s files
.s.o:
	@$(CC) $(ASFLAGS) -c -o $@ $<
	@echo $@

# Clean Target
clean:
	rm -f $(LIB_OBJS)
	rm -f $(USER_OBJS)
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).map
