
include $(APPDIR)/Make.defs
include $(SDKDIR)/Make.defs

PROGNAME = $(CONFIG_EXAMPLES_UWBINS_PROGNAME)
PRIORITY = $(CONFIG_EXAMPLES_UWBINS_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_UWBINS_STACKSIZE)
MODULE = $(CONFIG_EXAMPLES_UWBINS)

ASRCS =
CSRCS =
MAINSRC = uwbins_main.cxx
CXXSRCS = physical_sensor.cxx accel_sensor.cxx #gnss_sensor.cxx

# Sensing Example flags

CXXFLAGS += -D_POSIX
CXXFLAGS += -DUSE_MEMMGR_FENCE

include $(APPDIR)/Application.mk
