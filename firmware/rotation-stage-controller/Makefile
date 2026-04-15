# =============================================================================
# rotation-stage-controller — Makefile
#
# Target: STM32F103C8 (Blue Pill)
# Run from firmware/rotation-stage-controller/
#
# NOTE: This firmware currently uses STM32 Arduino APIs. The CMake build
# requires -DFETCH_STM32_HAL=ON and an Arduino compat layer (WIP).
# For production builds, use PlatformIO: pio run
# =============================================================================

BUILD_DIR     := build
CMAKE_GENERATOR ?= Ninja
CMAKE_EXTRA   ?=

.PHONY: all build configure clean flash help test pio

all: build  ## Default: build firmware

help:  ## Show this help
	@echo ""
	@echo "rotation-stage-controller Build System"
	@echo "======================================="
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?##' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  make %-12s %s\n", $$1, $$2}'
	@echo ""

configure:  ## Run CMake configure step
	@cmake -B $(BUILD_DIR) -S . \
		-G "$(CMAKE_GENERATOR)" \
		$(CMAKE_EXTRA)

build: configure  ## Build firmware
	@cmake --build $(BUILD_DIR) --parallel

clean:  ## Remove build directory
	@rm -rf $(BUILD_DIR) build-test
	@echo "Build directory removed."

flash: build  ## Flash via st-flash (requires st-link tools)
	@if command -v st-flash >/dev/null 2>&1; then \
		echo "Flashing via st-flash..."; \
		st-flash write $(BUILD_DIR)/rotation_stage.bin 0x8000000; \
	else \
		echo "st-flash not found. Install stlink tools or use PlatformIO."; \
		exit 1; \
	fi

pio:  ## Build with PlatformIO (recommended)
	@pio run

pio-upload:  ## Build and upload with PlatformIO
	@pio run -t upload

test:  ## Build and run host-native GoogleTest tests
	@cmake -B build-test -S test/host -G "$(CMAKE_GENERATOR)"
	@cmake --build build-test --parallel
	@cd build-test && ctest --output-on-failure
