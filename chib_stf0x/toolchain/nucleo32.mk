
# Directories for NUCLEO32 configuration

NUCLEO32_TOP           = ../..
NUCLEO32_TOOLCHAIN     = $(NUCLEO32_TOP)/toolchain/
OPENOCD_DIR           =  $(NUCLEO32_TOOLCHAIN)/openocd

# source directories
NUCLEO32_UTIL          = $(NUCLEO32_TOP)/src/util
NUCLEO32_SATSHELL      = $(NUCLEO32_TOP)/src/satshell
NUCLEO32_LTC2990       = $(NUCLEO32_TOP)/src/ltc2990
NUCLEO32_BOARDS        = $(NUCLEO32_TOP)/src/boards

# make rules
NUCLEO32_RULES         = $(OPENOCD_DIR)/openocd_stlinkv2-1.mk

ifeq ($(shell git diff-index --quiet HEAD $(NUCLEO32_TOP)/src ; echo $$?), 1)
INDEX_DIRTY = _INDEX_DIRTY
else
INDEX_DIRTY =
endif

VERSION_PREFIX = git-

NUCLEO32_VERSION = "\"$(VERSION_PREFIX)`git rev-parse --short HEAD`$(INDEX_DIRTY)\""

