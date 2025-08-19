#pragma once

#include "sensor.hpp"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <cmath>

#define AS5047_CPR             16384
#define AS5047_REG_ANGLECOM    0x3FFF
#define AS5047_SPI_MODE        1
#define AS5047_SPI_CLOCK_HZ    (15 * 1000 * 1000)
#define AS5047_CMD_READ_MASK   0x4000


#define PIN_CS     GPIO_NUM_44
#define PIN_CLK    GPIO_NUM_7
#define PIN_MISO   GPIO_NUM_8
#define PIN_MOSI   GPIO_NUM_9

#define SPI_HOST   SPI2_HOST
#define SPI_FREQ   (15 * 1000 * 1000)  // 15 MHz
#define CPR        16384
#define TWO_PI     (2.0f * M_PI)

#define ANGLECOM   0x3FFF


static inline void CS_LOW()  { gpio_set_level(PIN_CS, 0); }
static inline void CS_HIGH() { gpio_set_level(PIN_CS, 1); }


static uint16_t make_cmd(uint16_t addr)
{
    uint16_t frame = addr | 0x4000;
    uint16_t x = frame & 0x7FFF;
    x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;
    frame |= ((~x) & 0x0001) << 15;
    return frame;
}


static esp_err_t spi_transfer16_fast(spi_device_handle_t spi, uint16_t tx, uint16_t* rx, spi_transaction_t* t)
{
    uint16_t tx_swapped = __builtin_bswap16(tx);
    uint16_t rx_swapped = 0;

    
    t->tx_buffer = &tx_swapped;
    t->rx_buffer = &rx_swapped;

    CS_LOW();
    esp_err_t ret = spi_device_polling_transmit(spi, t);
    CS_HIGH();

    *rx = __builtin_bswap16(rx_swapped);
    return ret;
}

class AS5047 : public Sensor {
public:
    AS5047() = default;

    void on_init() override;
    void on_configure() override;
    void update() override;
    void reset() override;
    bool calibrate() override;

private:
    esp_err_t read_sensor(uint16_t* out);
    spi_device_handle_t spi;

};
