# micropython-nmea-c - tests.
#
#   make test                       C parser vs pynmea2 on the host (needs cc and pynmea2)
#   make test-unix MPY_DIR=<path>   build the MicroPython unix port with the module and
#                                   run tests/test_micropython.py (path without spaces:
#                                   make does not support them)
#   make clean

PYTHON ?= python3
BUILD  ?= build
MPY_DIR ?=

# The unix port is built without the features that need extra submodules.
UNIX_OPTS = VARIANT=standard MICROPY_PY_FFI=0 MICROPY_PY_BTREE=0 MICROPY_PY_SSL=0 MICROPY_SSL_MBEDTLS=0

.PHONY: test test-unix clean

test:
	$(PYTHON) tests/test_vs_pynmea2.py

# USER_C_MODULES must contain only this module: a link to the repository in $(BUILD)/modules.
test-unix:
	@test -n "$(MPY_DIR)" || { echo "usage: make test-unix MPY_DIR=/path/to/micropython"; exit 1; }
	mkdir -p $(BUILD)/modules
	ln -sfn $(CURDIR) $(BUILD)/modules/micropython-nmea-c
	$(MAKE) -C $(MPY_DIR)/mpy-cross
	$(MAKE) -C $(MPY_DIR)/ports/unix $(UNIX_OPTS) BUILD=$(abspath $(BUILD))/unix USER_C_MODULES=$(abspath $(BUILD))/modules
	$(abspath $(BUILD))/unix/micropython tests/test_micropython.py

clean:
	rm -rf $(BUILD)
