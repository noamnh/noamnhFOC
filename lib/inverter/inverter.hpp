#pragma once
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

#include "esp_err.h"
#include <cmath>
#include <algorithm> // For std::max and std::min

struct inverter_config{
int phase_a_high = 1;  // Phase A high-side
int phase_a_low = 2;   // Phase A low-side
int phase_b_high = 42;  // Phase B high-side
int phase_b_low = 41;   // Phase B low-side
int phase_c_high = 40;  // Phase C high-side
int phase_c_low = 39;   // Phase C low-side
int enable_pin = 38;    // Enable pin
} ;


class Inverter {
public:
    Inverter();
    ~Inverter();
    esp_err_t on_init();
    esp_err_t on_configure(inverter_config config);
    esp_err_t set_duty_cycle(float duty_a, float duty_b, float duty_c);

private:
    mcpwm_timer_handle_t timer; // ONE timer shared by all phases
    mcpwm_oper_handle_t operators[3];  // 3 operators for 3 phases // each operator controls one phase and several generators
    mcpwm_cmpr_handle_t comparators[3]; // 3 comparators for duty cycle control
    mcpwm_gen_handle_t generators[6];   // 6 generators (high + low side for each phase)
    
        // Configuration parameters
    static const uint32_t PWM_FREQ_HZ = 20000;  // 20kHz switching frequency
    static const uint32_t PWM_RESOLUTION = 1000; // 1000 ticks resolution (1000 steps per PWM cycle)
    static const uint32_t PWM_RESOLUTION_HZ = PWM_FREQ_HZ * PWM_RESOLUTION; // 20 MHz timer resolution
    static const uint32_t DEAD_TIME_US = 2;      // 2µs dead time preventing shoot-through(both high and low side on at the same time)


    esp_err_t create_timer();
    esp_err_t create_operators();
    esp_err_t create_generators();

    inverter_config config_;

};