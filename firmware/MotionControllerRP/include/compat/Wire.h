// =============================================================================
// Wire.h — Arduino TwoWire compatibility shim over Pico SDK hardware/i2c
// =============================================================================
#pragma once

#include "Arduino.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <cstdint>
#include <cstddef>

class TwoWire {
public:
    TwoWire()
        : m_i2c(i2c0), m_sda(0), m_scl(1) {}

    TwoWire(i2c_inst_t* inst, uint sda, uint scl)
        : m_i2c(inst), m_sda(sda), m_scl(scl) {}

    void setSDA(uint pin) { m_sda = pin; }
    void setSCL(uint pin) { m_scl = pin; }

    void begin() {
        i2c_init(m_i2c, m_clock);
        gpio_set_function(m_sda, GPIO_FUNC_I2C);
        gpio_set_function(m_scl, GPIO_FUNC_I2C);
        gpio_pull_up(m_sda);
        gpio_pull_up(m_scl);
    }

    void setClock(uint32_t freq) {
        m_clock = freq;
        i2c_set_baudrate(m_i2c, freq);
    }

    void beginTransmission(uint8_t addr) {
        m_addr   = addr;
        m_tx_len = 0;
    }

    size_t write(uint8_t data) {
        if (m_tx_len < sizeof(m_tx_buf)) {
            m_tx_buf[m_tx_len++] = data;
            return 1;
        }
        return 0;
    }

    // Returns Arduino-compatible status: 0 = success, 2 = NACK addr, 4 = other
    uint8_t endTransmission(bool stop = true) {
        int ret = i2c_write_blocking(m_i2c, m_addr, m_tx_buf, m_tx_len, !stop);
        m_tx_len = 0;
        if (ret == PICO_ERROR_GENERIC) return 2;
        if (ret < 0)                   return 4;
        return 0;
    }

    uint8_t requestFrom(uint8_t addr, uint8_t count) {
        m_rx_idx = 0;
        m_rx_len = 0;
        if (count > sizeof(m_rx_buf)) count = sizeof(m_rx_buf);
        int ret = i2c_read_blocking(m_i2c, addr, m_rx_buf, count, false);
        if (ret < 0) return 0;
        m_rx_len = static_cast<uint8_t>(ret);
        return m_rx_len;
    }

    int available() const {
        return m_rx_len - m_rx_idx;
    }

    int read() {
        if (m_rx_idx < m_rx_len) return m_rx_buf[m_rx_idx++];
        return -1;
    }

private:
    i2c_inst_t* m_i2c;
    uint        m_sda;
    uint        m_scl;
    uint32_t    m_clock  = 100000;
    uint8_t     m_addr   = 0;

    uint8_t     m_tx_buf[64]{};
    uint8_t     m_tx_len = 0;

    uint8_t     m_rx_buf[64]{};
    uint8_t     m_rx_len = 0;
    uint8_t     m_rx_idx = 0;
};

extern TwoWire Wire;
