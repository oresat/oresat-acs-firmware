
# Directories for NUCLEO64 configuration

NUCLEO64_TOP           = ../..
NUCLEO64_TOOLCHAIN     = $(NUCLEO64_TOP)/toolchain
OPENOCD_DIR            = $(NUCLEO64_TOOLCHAIN)/openocd

# source directories
NUCLEO64_UTIL          = $(NUCLEO64_TOP)/src/util
NUCLEO64_LTC2990       = $(NUCLEO64_TOP)/src/ltc2990
NUCLEO64_SX1236        = $(NUCLEO64_TOP)/src/sx1236
NUCLEO64_SOLARV1       = $(NUCLEO64_TOP)/src/solar_v1
NUCLEO64_SATSHELL      = $(NUCLEO64_TOP)/src/satshell
NUCLEO64_BOARDS        = $(NUCLEO64_TOP)/src/boards

# make rules
NUCLEO64_RULES         = $(OPENOCD_DIR)/openocd_stlinkv2-1.mk

ifeq ($(shell git diff-index --quiet HEAD $(NUCLEO64_TOP)/src ; echo $$?), 1)
INDEX_DIRTY = _INDEX_DIRTY
else
INDEX_DIRTY =
endif

VERSION_PREFIX = git-

NUCLEO64_VERSION = "\"$(VERSION_PREFIX)`git rev-parse --short HEAD`$(INDEX_DIRTY)\""

