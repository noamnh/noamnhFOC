#pragma once

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "utils.hpp"

#define I2C_MASTER_SCL_IO 7         // GPIO for SCL line
#define I2C_MASTER_SDA_IO 6         // GPIO for SDA line
#define I2C_MASTER_NUM I2C_NUM_0    // I2C port number for master
#define I2C_MASTER_FREQ_HZ 800000   // I2C clock frequency
#define I2C_MASTER_TX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define AS5600_ADDR 0x36            // I2C address of AS5600
#define AS5600_RAW_ANGLE_L 0x0D        // Raw angle (7:0)
#define AS5600_RAW_ANGLE_H 0x0C        // Raw angle (11:8)

class EncoderI2C {

    public:
    EncoderI2C();
    EncoderI2C(int sda, int scl, uint32_t addr, uint32_t freq);
    void configure(int sda, int scl, uint32_t addr, uint32_t freq);
    void init();
    void read();
    void write();
    float getAngle();
    float getAngleDegrees();
    float getVelocity();
    float getRawAngle();
    float getPrevAngle();
    float getAccumulatedAngle();
    int getSensorDirection(){return sensor_direction_;}
    void setSensorDirection(int direction){sensor_direction_ = direction;}
    void setOffset(float offset){
        offset_ = offset;
        accumulated_angle_ = 0;
        prev_angle_ = 0;}

    private:
    int sda_;
    int scl_;
    int sensor_direction_ = 1;
    float angle_;
    float velocity_;
    float prev_angle_;
    float offset_;
    float accumulated_angle_;
    float prev_angle_time_;
    float prev_angle_velocity_;
    float prev_angle_velocity_time_;
    float min_elapsed_time_ = 0.0001;
    uint32_t addr_;
    uint32_t freq_;
    esp_err_t read_register_(uint8_t reg, uint8_t *data);
};