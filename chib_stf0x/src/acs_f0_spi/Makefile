##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
	# This will make very big object
	# USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16
	USE_OPT = -ggdb -fomit-frame-pointer -falign-functions=16

	# This is smaller object
	# USE_OPT = -Os -fomit-frame-pointer -falign-functions=16
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
	USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
	USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
	USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
	USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
	USE_LTO = yes
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
	USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
	USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
	USE_SMART_BUILD = no
	# disabled because this throws errors since it can't find the conf files
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
	USE_PROCESS_STACKSIZE = 0x200
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
	USE_EXCEPTIONS_STACKSIZE = 0x200
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
	USE_FPU = no
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = acs_f0_spi

# Imported source files and paths
CONF_DIR  = conf
TOOLCHAIN = ../../toolchain
CHIBIOS   = ../../../ChibiOS
BOARD     = ST_NUCLEO32_F042K6
BOARDDIR  = ../boards/$(BOARD)

# Nucleo32 specific files
include $(TOOLCHAIN)/nucleo32.mk

include $(BOARDDIR)/board.mk

# Startup files
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32f0xx.mk
# include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f0xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F0xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v6m.mk
# include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v6m.mk

# Other files (optional).
include $(CHIBIOS)/test/rt/test.mk

# Define linker script file here
LDSCRIPT= $(STARTUPLD)/STM32F042x6.ld

# Add application source directories here
APP_SRCS = $(wildcard $(NUCLEO32_UTIL)/*.c)
APP_SRCS += $(wildcard $(NUCLEO32_BOARDS)/*.c)

CSRC = $(STARTUPSRC) \
       $(KERNSRC) \
       $(PORTSRC) \
       $(OSALSRC) \
       $(HALSRC) \
       $(PLATFORMSRC) \
       $(BOARDSRC) \
       $(CHIBIOS)/os/hal/lib/streams/memstreams.c \
       $(CHIBIOS)/os/hal/lib/streams/chprintf.c \
       $(APP_SRCS) \
       main.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC =
ASMXSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(CHIBIOS)/os/license \
	 $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
	 $(HALINC) $(PLATFORMINC) $(BOARDINC) $(TESTINC) \
	 $(CHIBIOS)/os/hal/lib/streams $(CHIBIOS)/os/various \
	 $(CONF_DIR) \
	 $(NUCLEO32_UTIL)/include \
	 $(NUCLEO32_BOARDS)/include \
	 $(CHIBIOS)/os/various

# $(NUCLEO32_SATSHELL)/include \

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m0

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
# CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

CWARN = -Wall -Wextra -Wstrict-prototypes  -Wdisabled-optimization \
	-Wdouble-promotion -Wformat=2 -Wfloat-equal \
	-Waggressive-loop-optimizations \
	-Waggregate-return -Wlogical-op \
	-Wno-implicit-fallthrough

#-Wmissing-include-dirs

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DGIT_COMMIT_VERSION=$(NUCLEO32_VERSION)
#UDEFS += -DDBG_MSG_ENABLE=1

ORESAT_CAN_ADDRESS=0xA0
UDEFS += -DMY_CAN_ADDRESS=$(ORESAT_CAN_ADDRESS)

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
# ULIBS = -lm
ULIBS =

#
# End of user defines
##############################################################################
RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC
# RULESPATH = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk

# PORTRULESPATH = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk
# Arm v6m is for ARM M0
# https://community.arm.com/processors/b/blog/posts/armv6-m-vs-armv7-m---unpacking-the-microcontrollers
# include $(PORTRULESPATH)/port_v6m.mk

include $(NUCLEO32_RULES)

