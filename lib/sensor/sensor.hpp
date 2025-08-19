
#pragma once

#ifndef SENSOR_HPP  
#define SENSOR_HPP

#include "esp_log.h"
#include "utils.h"
#define _2PI                                6.28318530718f


typedef struct {
    uint16_t ticks;
    float raw_angle;
    float prev_angle; // Previous angle for velocity calculation
    float multiturn_angle;
    float velocity;
    uint32_t timestamp; // Timestamp in microseconds
} sensor_state_t;

typedef struct {
    bool is_calibrated;
    uint32_t error;
} sensor_status_t;

typedef struct {
    float offset; // Offset for angle correction
    uint16_t resolution; // Resolution of the sensor in ticks
} sensor_config_t;


class Sensor {
public:
    Sensor() = default;

    virtual void on_init() = 0;
    virtual void on_configure() = 0;
    virtual void update() = 0;
    virtual void reset() = 0;
    virtual bool calibrate() = 0;
    
    float get_raw_angle() {
        // state_.raw_angle = (state_.ticks* _2PI) / config_.resolution;
        return state_.raw_angle;
    } // get the raw single turn angle
    uint16_t get_ticks() const {
        return state_.ticks;
    } // get the raw ticks
    float get_multiturn_angle() const {
        return state_.multiturn_angle;
    } // get the multiturn angle
    float get_velocity() const {
        return state_.velocity;
    } // get the velocity

    
    protected:
    sensor_state_t state_;
    sensor_status_t status_;
    sensor_config_t config_;

    private:


};

#endif // SENSOR_HPP
