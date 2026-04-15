# =============================================================================
# MotionControllerRP — Top-Level Makefile
#
# Convenience targets for CMake/Pico SDK builds.
#
# Prerequisites:
#   - cmake, ninja (or make), arm-none-eabi-gcc toolchain
#   - PICO_SDK_PATH set to your Pico SDK checkout, OR pass
#     -DPICO_SDK_FETCH_FROM_GIT=ON to have CMake download it.
#   - picotool (optional, for USB flash)
#
# Install on MSYS2/Windows:
#   pacman -S mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja
#   pacman -S mingw-w64-ucrt-x86_64-arm-none-eabi-gcc
#
# Install on Linux/macOS:
#   sudo apt install cmake ninja-build gcc-arm-none-eabi   # Debian/Ubuntu
#   brew install cmake ninja arm-none-eabi-gcc              # macOS
# =============================================================================

# --- Configuration -----------------------------------------------------------

BUILD_DIR     := build
BINARY_NAME   := motion_controller
UF2_FILE      := $(BUILD_DIR)/$(BINARY_NAME).uf2
ELF_FILE      := $(BUILD_DIR)/$(BINARY_NAME).elf

# CMake generator: prefer Ninja if available, fall back to Unix Makefiles
CMAKE_GENERATOR ?= Ninja

# Serial port for monitoring (override: make monitor PORT=/dev/ttyACM0)
PORT          ?= auto
BAUD          ?= 921600

# Extra CMake flags (e.g. -DPICO_SDK_FETCH_FROM_GIT=ON)
CMAKE_EXTRA   ?=

# --- Targets -----------------------------------------------------------------

.PHONY: all build configure clean flash monitor run help test

all: build  ## Default: build firmware

help:  ## Show this help
	@echo ""
	@echo "MotionControllerRP Build System"
	@echo "==============================="
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?##' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  make %-12s %s\n", $$1, $$2}'
	@echo ""
	@echo "Environment variables:"
	@echo "  PICO_SDK_PATH    Path to the Pico SDK (required unless fetching)"
	@echo "  CMAKE_EXTRA      Extra flags for cmake configure step"
	@echo "  PORT             Serial port for monitor (default: auto-detect)"
	@echo "  BAUD             Serial baud rate (default: 921600)"
	@echo ""

configure:  ## Run CMake configure step
	@cmake -B $(BUILD_DIR) -S . \
		-G "$(CMAKE_GENERATOR)" \
		$(CMAKE_EXTRA)

build: configure  ## Build firmware (produces .uf2 and .elf)
	@cmake --build $(BUILD_DIR) --parallel
	@echo ""
	@echo "Build complete:"
	@echo "  ELF: $(ELF_FILE)"
	@echo "  UF2: $(UF2_FILE)"

clean:  ## Remove build directory
	@rm -rf $(BUILD_DIR)
	@echo "Build directory removed."

flash: build  ## Flash firmware to Pico 2 via picotool
	@if command -v picotool >/dev/null 2>&1; then \
		echo "Flashing via picotool..."; \
		picotool load -f $(UF2_FILE); \
		picotool reboot; \
	else \
		echo ""; \
		echo "picotool not found. Flash manually:"; \
		echo "  1. Hold BOOTSEL on the Pico 2 and plug in USB"; \
		echo "  2. Copy $(UF2_FILE) to the RPI-RP2 drive"; \
		echo ""; \
		echo "Or install picotool:"; \
		echo "  https://github.com/raspberrypi/picotool"; \
		exit 1; \
	fi

monitor:  ## Open serial monitor (requires Python + pyserial)
	@if command -v python3 >/dev/null 2>&1; then \
		python3 -m serial.tools.miniterm $(PORT) $(BAUD); \
	elif command -v python >/dev/null 2>&1; then \
		python -m serial.tools.miniterm $(PORT) $(BAUD); \
	else \
		echo "Python not found. Install pyserial:  pip install pyserial"; \
		exit 1; \
	fi

run: flash monitor  ## Flash then open serial monitor

test:  ## Build and run host-native GoogleTest tests
	@cmake -B build-test -S test/host -G "$(CMAKE_GENERATOR)"
	@cmake --build build-test --parallel
	@cd build-test && ctest --output-on-failure
