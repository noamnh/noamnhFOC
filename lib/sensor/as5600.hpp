#include "sensor.hpp"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define I2C_MASTER_SCL_IO 6       // GPIO for SCL line
#define I2C_MASTER_SDA_IO 5         // GPIO for SDA line
#define I2C_MASTER_NUM I2C_NUM_0    // I2C port number for master
#define I2C_MASTER_FREQ_HZ 800000   // I2C clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define AS5600_ADDR 0x36            // I2C address of AS5600
#define AS5600_RAW_ANGLE_L 0x0D        // Raw angle (7:0)
#define AS5600_RAW_ANGLE_H 0x0C        // Raw angle (11:8)
#pragma once

class AS5600 : public Sensor {
public:
    AS5600();

    void on_init() override;

    void on_configure() override;

    void update() override;

    void reset() override;

    bool calibrate() override;

    float get_raw_angle() const override; // get the raw single turn angle

    float get_ticks() const override; // get the raw ticks

    float get_multiturn_angle() const override; // get the multiturn angle

    float get_velocity() const override; // get the velocity

private:    

    esp_err_t read_register_(uint8_t reg, uint8_t *data);
};