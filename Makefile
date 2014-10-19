PROJECT = traffic

OUTDIR = build
EXECUTABLE = $(OUTDIR)/$(PROJECT).elf
BIN_IMAGE = $(OUTDIR)/$(PROJECT).bin
HEX_IMAGE = $(OUTDIR)/$(PROJECT).hex

STM32_LIB = CORTEX_M4F_STM32F4/Libraries/STM32F4xx_StdPeriph_Driver

# set the path to STM32F429I-Discovery firmware package
STDP ?= ../STM32F429I-Discovery_FW_V1.0.1

# Toolchain configurations
CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size

# Cortex-M4 implements the ARMv7E-M architecture
CPU = cortex-m4
CFLAGS = -mcpu=$(CPU) -march=armv7e-m -mtune=cortex-m4
CFLAGS += -mlittle-endian -mthumb
# Need study
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -O0

define get_library_path
    $(shell dirname $(shell $(CC) $(CFLAGS) -print-file-name=$(1)))
endef
LDFLAGS += -L $(call get_library_path,libc.a)
LDFLAGS += -L $(call get_library_path,libgcc.a)

# Basic configurations
CFLAGS += -g -std=c99 -Wall

# Optimizations
CFLAGS += -O3 -ffast-math \
		  -ffunction-sections -fdata-sections \
		  -Wl,--gc-sections \
		  -fno-common \
		  --param max-inline-insns-single=1000

# specify STM32F429
CFLAGS += -DSTM32F429_439xx

# to run from FLASH
CFLAGS += -DVECT_TAB_FLASH
LDFLAGS += -T CORTEX_M4F_STM32F4/stm32f429zi_flash.ld

#files
SRCDIR = src\
		 portable/GCC/ARM_CM4F\
		 src/traffic
		 
INCDIR = CORTEX_M4F_STM32F4 \
		 include \
		 portable/GCC/ARM_CM4F \
		 CORTEX_M4F_STM32F4/board \
		 CORTEX_M4F_STM32F4/Libraries/CMSIS/Device/ST/STM32F4xx/Include \
		 CORTEX_M4F_STM32F4/Libraries/CMSIS/Include \
		 $(STM32_LIB)/inc \
		 Utilities/STM32F429I-Discovery \
		 src/traffic/include

# STARTUP FILE
SRC += CORTEX_M4F_STM32F4/startup_stm32f429_439xx.c

# STM32F4xx_StdPeriph_Driver
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -D"assert_param(expr)=((void)0)"

#My restart
SRC += \
      CORTEX_M4F_STM32F4/main.c \
      CORTEX_M4F_STM32F4/startup/system_stm32f4xx.c \
      #CORTEX_M4F_STM32F4/stm32f4xx_it.c \

SRC += $(wildcard $(addsuffix /*.c,$(SRCDIR))) \
	  $(wildcard $(addsuffix /*.s,$(SRCDIR)))

SRC += portable/MemMang/heap_1.c

SRC +=  $(STM32_LIB)/src/misc.c \
		$(STM32_LIB)/src/stm32f4xx_gpio.c \
		$(STM32_LIB)/src/stm32f4xx_rcc.c \
		$(STM32_LIB)/src/stm32f4xx_usart.c \
		$(STM32_LIB)/src/stm32f4xx_syscfg.c \
		$(STM32_LIB)/src/stm32f4xx_i2c.c \
		$(STM32_LIB)/src/stm32f4xx_dma.c \
		$(STM32_LIB)/src/stm32f4xx_spi.c \
		$(STM32_LIB)/src/stm32f4xx_exti.c \
		$(STM32_LIB)/src/stm32f4xx_dma2d.c \
		$(STM32_LIB)/src/stm32f4xx_ltdc.c \
		$(STM32_LIB)/src/stm32f4xx_fmc.c \
		$(STM32_LIB)/src/stm32f4xx_rng.c \
		Utilities/STM32F429I-Discovery/stm32f429i_discovery.c \
		Utilities/STM32F429I-Discovery/stm32f429i_discovery_sdram.c \
		Utilities/STM32F429I-Discovery/stm32f429i_discovery_lcd.c \
		Utilities/STM32F429I-Discovery/stm32f429i_discovery_ioe.c

OBJS += $(addprefix $(OUTDIR)/,$(patsubst %.s,%.o,$(SRC:.c=.o)))

CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += $(addprefix -I,$(INCDIR))

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@
	$(OBJCOPY) -O ihex $^ $(HEX_IMAGE)
	$(OBJDUMP) -h -S -D $(EXECUTABLE) > $(OUTDIR)/$(PROJECT).lst
	$(SIZE) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJS)
	$(LD) -o $@ $(OBJS) \
		--start-group $(LIBS) --end-group \
		$(LDFLAGS)

$(OUTDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(OUTDIR)/%.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

flash:
	st-flash write $(BIN_IMAGE) 0x8000000

openocd_flash:
	openocd \
	-f board/stm32f429discovery.cfg \
	-c "init" \
	-c "reset init" \
	-c "flash probe 0" \
	-c "flash info 0" \
	-c "flash write_image erase $(BIN_IMAGE)  0x08000000" \
	-c "reset run" -c shutdown

.PHONY: clean
clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm -rf $(HEX_IMAGE)
	rm -f $(OBJS)
	rm -f $(OUTDIR)/$(PROJECT).lst
