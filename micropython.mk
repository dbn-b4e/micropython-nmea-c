# micropython-nmea-c - module `nmea` for make-based ports (stm32, unix, rp2 with make...).
# Build with USER_C_MODULES=<directory containing this repository>.

NMEA_MOD_DIR := $(USERMOD_DIR)

SRC_USERMOD += $(NMEA_MOD_DIR)/modnmea.c
SRC_USERMOD += $(NMEA_MOD_DIR)/src/nmea_core.c
SRC_USERMOD += $(NMEA_MOD_DIR)/src/nmea_vendor.c

CFLAGS_USERMOD += -I$(NMEA_MOD_DIR)
