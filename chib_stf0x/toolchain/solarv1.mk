
# Directories for SOLARV1 configuration

SOLARV1_TOP           = ../..
SOLARV1_TOOLCHAIN     = $(SOLARV1_TOP)/toolchain/
OPENOCD_DIR           = $(SOLARV1_TOOLCHAIN)/openocd

# source directories
SOLARV1_UTIL          = $(SOLARV1_TOP)/src/util
SOLARV1_LTC2990       = $(SOLARV1_TOP)/src/ltc2990
SOLARV1_SOLARV1       = $(SOLARV1_TOP)/src/solar_v1
SOLARV1_SATSHELL      = $(SOLARV1_TOP)/src/satshell
SOLARV1_BOARDS        = $(SOLARV1_TOP)/src/boards

# make rules
SOLARV1_RULES         = $(OPENOCD_DIR)/openocd_stlinkv2.mk

ifeq ($(shell git diff-index --quiet HEAD $(SOLARV1_TOP)/src ; echo $$?), 1)
INDEX_DIRTY = _INDEX_DIRTY
else
INDEX_DIRTY =
endif

VERSION_PREFIX = git-

SOLARV1_VERSION = "\"$(VERSION_PREFIX)`git rev-parse --short HEAD`$(INDEX_DIRTY)\""

