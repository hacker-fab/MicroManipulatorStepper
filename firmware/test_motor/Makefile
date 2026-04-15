# =============================================================================
# test_motor — Makefile
#
# Convenience targets for CMake/Pico SDK builds.
# Run from firmware/test_motor/
# =============================================================================

BUILD_DIR     := build
CMAKE_GENERATOR ?= Ninja
CMAKE_EXTRA   ?=
PORT          ?= auto
BAUD          ?= 115200

# Default target to flash (override: make flash TARGET=encoder)
TARGET        ?= blink

.PHONY: all build configure clean flash monitor run help test

all: build  ## Default: build all test firmware

help:  ## Show this help
	@echo ""
	@echo "test_motor Build System"
	@echo "======================="
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?##' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  make %-12s %s\n", $$1, $$2}'
	@echo ""
	@echo "Targets: blink, encoder, stepper, stepper_and_encoder, one_shot_encoder, peripheral_test"
	@echo "  make flash TARGET=encoder    Flash a specific test program"
	@echo ""

configure:  ## Run CMake configure step
	@cmake -B $(BUILD_DIR) -S . \
		-G "$(CMAKE_GENERATOR)" \
		$(CMAKE_EXTRA)

build: configure  ## Build all test firmware
	@cmake --build $(BUILD_DIR) --parallel
	@echo ""
	@echo "Build complete. UF2 files in $(BUILD_DIR)/"

clean:  ## Remove build directory
	@rm -rf $(BUILD_DIR)
	@echo "Build directory removed."

flash: build  ## Flash a test program (make flash TARGET=blink)
	@if command -v picotool >/dev/null 2>&1; then \
		echo "Flashing $(TARGET) via picotool..."; \
		picotool load -f $(BUILD_DIR)/$(TARGET).uf2; \
		picotool reboot; \
	else \
		echo "picotool not found. Copy $(BUILD_DIR)/$(TARGET).uf2 to the RPI-RP2 drive."; \
		exit 1; \
	fi

monitor:  ## Open serial monitor
	@if command -v python3 >/dev/null 2>&1; then \
		python3 -m serial.tools.miniterm $(PORT) $(BAUD); \
	elif command -v python >/dev/null 2>&1; then \
		python -m serial.tools.miniterm $(PORT) $(BAUD); \
	else \
		echo "Python not found. Install pyserial: pip install pyserial"; \
		exit 1; \
	fi

run: flash monitor  ## Flash then open serial monitor

test:  ## Build and run host-native GoogleTest tests
	@cmake -B build-test -S test/host -G "$(CMAKE_GENERATOR)"
	@cmake --build build-test --parallel
	@cd build-test && ctest --output-on-failure
