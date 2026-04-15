// =============================================================================
// SPI.h — Arduino SPI compatibility shim over Pico SDK hardware/spi
// =============================================================================
#pragma once

#include "Arduino.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <cstdint>

#define MSBFIRST 1
#define LSBFIRST 0
#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

class SPISettings {
public:
    SPISettings() : m_clock(1000000), m_bitOrder(MSBFIRST), m_dataMode(SPI_MODE0) {}
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
        : m_clock(clock), m_bitOrder(bitOrder), m_dataMode(dataMode) {}
    uint32_t m_clock;
    uint8_t  m_bitOrder;
    uint8_t  m_dataMode;
};

class SPIClass {
public:
    SPIClass() : m_spi(spi0) {}

    void setSCK(uint pin) { m_sck = pin; }
    void setRX(uint pin)  { m_miso = pin; }
    void setTX(uint pin)  { m_mosi = pin; }

    void begin() {
        spi_init(m_spi, 1000000);
        gpio_set_function(m_sck,  GPIO_FUNC_SPI);
        gpio_set_function(m_miso, GPIO_FUNC_SPI);
        gpio_set_function(m_mosi, GPIO_FUNC_SPI);
    }

    void beginTransaction(SPISettings settings) {
        spi_set_baudrate(m_spi, settings.m_clock);
        uint cpol = (settings.m_dataMode & 0x02) ? 1 : 0;
        uint cpha = (settings.m_dataMode & 0x01) ? 1 : 0;
        spi_set_format(m_spi, 8,
                       static_cast<spi_cpol_t>(cpol),
                       static_cast<spi_cpha_t>(cpha),
                       SPI_MSB_FIRST);
    }

    uint8_t transfer(uint8_t data) {
        uint8_t rx;
        spi_write_read_blocking(m_spi, &data, &rx, 1);
        return rx;
    }

    void endTransaction() {}

    void end() {
        spi_deinit(m_spi);
    }

private:
    spi_inst_t* m_spi;
    uint m_sck  = 2;
    uint m_miso = 0;
    uint m_mosi = 3;
};

extern SPIClass SPI;
