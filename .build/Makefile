# project name (generate the executable file with this name)
TARGET = executable

# currently directory of each file
SRCDIR 		= ../user/src
OBJDIR 		= ./obj/user
BINDIR 		= ../bin/debug
SYSDIR		= /home/kleber/Downloads/mbed-master/build/mbed

# general definition of the files
STATICLIB 	= $(LIBDIR)/lib$(TARGET).a
SOURCES 	:= $(wildcard $(SRCDIR)/*.cpp)
INCLUDES 	:= $(wildcard $(INCLUDEDIR)/*.h)
OBJECTS 	:= $(SOURCES:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)
SYS_OBJECTS := $(wildcard $(SYSDIR)/TARGET_MTS_MDOT_F405RG/TOOLCHAIN_GCC_ARM/*.o)
INCLUDE_PATHS = -I../user/include -I$(SYSDIR) -I$(SYSDIR)/TARGET_MTS_MDOT_F405RG -I$(SYSDIR)/TARGET_MTS_MDOT_F405RG/TARGET_STM -I$(SYSDIR)/TARGET_MTS_MDOT_F405RG/TARGET_STM/TARGET_STM32F4 -I$(SYSDIR)/TARGET_MTS_MDOT_F405RG/TARGET_STM/TARGET_STM32F4/TARGET_MTS_MDOT_F405RG -I$(SYSDIR)/TARGET_MTS_MDOT_F405RG/TOOLCHAIN_GCC_ARM
LIBRARY_PATHS = -L$(SYSDIR)/TARGET_MTS_MDOT_F405RG/TOOLCHAIN_GCC_ARM
LINKER_SCRIPT = $(SYSDIR)/TARGET_MTS_MDOT_F405RG/TOOLCHAIN_GCC_ARM/STM32F405.ld
CC_SYMBOLS = -DTARGET_RTOS_M4_M7 -DTARGET_MTS_MDOT_F405RG -DTOOLCHAIN_GCC_ARM -DTOOLCHAIN_GCC -DMBED_BUILD_TIMESTAMP=1453841063.57 -D__CORTEX_M4 -DTARGET_CORTEX_M -DLSI_VALUE=32000 -D__FPU_PRESENT=1 -DTARGET_FF_ARDUINO -D__MBED__=1 -DTARGET_M4 -DTARGET_STM -DTARGET_STM32F407 -DTARGET_STM32F4 -DTARGET_STM32F407VG -DARM_MATH_CM4

# compiler version in use
CC = arm-none-eabi-

CPU =	-mcpu=cortex-m4 \
		-mthumb \
		-mfpu=fpv4-sp-d16 \
		-mfloat-abi=softfp

# compiling flags here
CFLAGS 	=	-c \
			-g \
			-fno-common \
			-fmessage-length=0 \
			-Wall \
			-Wextra \
			-fno-exceptions \
			-ffunction-sections \
			-fdata-sections \
			-fomit-frame-pointer \
			-MMD \
			-MP \
			-ggdb3 \
			$(INCLUDE_PATHS) \

# linking flags here
LDFLAGS =	-Wl,--gc-sections \
			--specs=nano.specs \
			-u _printf_float \
			-u _scanf_float \
			-Wl,--wrap,main \
			-Wl,-Map=$(TARGET).map,--cref \
			-T$(LINKER_SCRIPT) \
			$(LIBRARY_PATHS) \

# list of libraries used
LIBS = -lmbed -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys

# clean process
rm = rm -f

# converts the ELF file into a binary file
$(BINDIR)/$(TARGET).bin: $(BINDIR)/$(TARGET).elf
	@$(CC)objcopy -O binary $< $@

# links the objects creating the static library and then the executable
$(BINDIR)/$(TARGET).elf: $(OBJECTS)
	@$(CC)gcc $(CPU) $(LDFLAGS) -o $@ $(SYS_OBJECTS) $(OBJECTS) $(LIBS)
	@echo "Linking complete!"

# compiles the sources creating the objects
$(OBJECTS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp $(INCLUDES) Makefile
	@$(CC)g++ $(CPU) $(CFLAGS) $(CC_SYMBOLS) -std=gnu++98 -fno-rtti -c $< -o $@
	@echo "Compiled "$<" successfully!"

# cleanup
clean: clean_bin clean_obj

# cleans the executable only
clean_bin:
	@$(rm) $(BINDIR)/$(TARGET).elf $(BINDIR)/$(TARGET).bin
	@echo "Executable removed!"

# cleans the objects only
clean_obj:
	@$(rm) $(OBJECTS)
	@echo "Objects removed!"

