################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../clksys_driver.c \
../DataInSlave.c \
../H2A_SECOM_MotorDriver.c \
../md_9dof.c \
../md_core_analog.c \
../md_readbussensors.c \
../md_serial.c \
../md_ticktimer.c \
../util.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
clksys_driver.o \
DataInSlave.o \
H2A_SECOM_MotorDriver.o \
md_9dof.o \
md_core_analog.o \
md_readbussensors.o \
md_serial.o \
md_ticktimer.o \
util.o

OBJS_AS_ARGS +=  \
clksys_driver.o \
DataInSlave.o \
H2A_SECOM_MotorDriver.o \
md_9dof.o \
md_core_analog.o \
md_readbussensors.o \
md_serial.o \
md_ticktimer.o \
util.o

C_DEPS +=  \
clksys_driver.d \
DataInSlave.d \
H2A_SECOM_MotorDriver.d \
md_9dof.d \
md_core_analog.d \
md_readbussensors.d \
md_serial.d \
md_ticktimer.d \
util.d

C_DEPS_AS_ARGS +=  \
clksys_driver.d \
DataInSlave.d \
H2A_SECOM_MotorDriver.d \
md_9dof.d \
md_core_analog.d \
md_readbussensors.d \
md_serial.d \
md_ticktimer.d \
util.d

OUTPUT_FILE_PATH +=H2A_SECOM_MotorDriver.elf

OUTPUT_FILE_PATH_AS_ARGS +=H2A_SECOM_MotorDriver.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler



















./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.9.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\XMEGAA_DFP\1.1.68\include"  -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g -Wall -mmcu=atxmega256a3u -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\XMEGAA_DFP\1.1.68\gcc\dev\atxmega256a3u" -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.9.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="H2A_SECOM_MotorDriver.map" -Wl,-u,vfprintf -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atxmega256a3u -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\XMEGAA_DFP\1.1.68\gcc\dev\atxmega256a3u" -lprintf_flt -lm  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "H2A_SECOM_MotorDriver.elf" "H2A_SECOM_MotorDriver.hex"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "H2A_SECOM_MotorDriver.elf" "H2A_SECOM_MotorDriver.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "H2A_SECOM_MotorDriver.elf" > "H2A_SECOM_MotorDriver.lss"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "H2A_SECOM_MotorDriver.elf" "H2A_SECOM_MotorDriver.srec"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "H2A_SECOM_MotorDriver.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "H2A_SECOM_MotorDriver.elf" "H2A_SECOM_MotorDriver.a" "H2A_SECOM_MotorDriver.hex" "H2A_SECOM_MotorDriver.lss" "H2A_SECOM_MotorDriver.eep" "H2A_SECOM_MotorDriver.map" "H2A_SECOM_MotorDriver.srec" "H2A_SECOM_MotorDriver.usersignatures"
	