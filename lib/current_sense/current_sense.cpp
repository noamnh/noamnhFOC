#include "current_sense.hpp"
#include <math.h>

CurrentSense::CurrentSense() {
    // Constructor
}

void CurrentSense::on_configure(const CurrentSenseConfig& config) {
    // Configure the current sense
    config_ = config;
    volts_to_amps_ratio_ = 1/config_.shunt_resistance / config_.gain;
    printf("CurrentSense configured\n");
    printf("Shunt resistance: %f, Gain: %f\n", config_.shunt_resistance, config_.gain);
    printf("Volts to Amps ratio: %f\n", volts_to_amps_ratio_);
}

void CurrentSense::on_init() {
    adc1_config_width(ADC_WIDTH);

    // Configure ADC channels
    adc1_config_channel_atten((adc1_channel_t)config_.u_pin, ADC_ATTEN);
    adc1_config_channel_atten((adc1_channel_t)config_.v_pin, ADC_ATTEN);
    adc1_config_channel_atten((adc1_channel_t)config_.w_pin, ADC_ATTEN);

}

void CurrentSense::on_activate() {
    // Activate the current sense
}

void CurrentSense::on_deactivate() {
    // Deactivate the current sense
}

void CurrentSense::sample() {
    // Read raw ADC values
    float raw_u = adc1_get_raw((adc1_channel_t)config_.u_pin);
    float raw_v = adc1_get_raw((adc1_channel_t)config_.v_pin);
    float raw_w = adc1_get_raw((adc1_channel_t)config_.w_pin);

    // Precompute voltage scaling factor
    constexpr float ADC_SCALE = 3.3f / 4095.0f;

    // Convert raw ADC values to voltage (in volts)
    float voltage_u = raw_u * ADC_SCALE - offset_u_;
    float voltage_v = raw_v * ADC_SCALE - offset_v_;
    float voltage_w = raw_w * ADC_SCALE - offset_w_;

    // Precompute current conversion factor
    float current_factor = 1.0f / (config_.shunt_resistance * config_.gain);

    // Compute currents using precomputed factor
     iu_ = voltage_u * current_factor;
     iv_ = voltage_v * current_factor;
     iw_ = voltage_w * current_factor;

    
}


void CurrentSense::calibrate() {

    float offset_u = 0;
    float offset_v = 0;
    float offset_w = 0;

    for (int i = 0; i < 1000; i++) {
        float raw_u = getAveragedADC(config_.u_pin);
        float raw_v = getAveragedADC(config_.v_pin);
        float raw_w = getAveragedADC(config_.w_pin);

        // Convert to voltage (in mV)
        uint32_t voltage_u = (raw_u * 3300) / 4095;  // Scale to 3.3V
        uint32_t voltage_v = (raw_v * 3300) / 4095;
        uint32_t voltage_w = (raw_w * 3300) / 4095;
        
        offset_u += voltage_u;
        offset_v += voltage_v;
        offset_w += voltage_w;
        
    }
    offset_u /= 1000;
    offset_v /= 1000;
    offset_w /= 1000;
    set_offsets(offset_u/1000, offset_v/1000, offset_w/1000);
    printf("Offsets: %f, %f, %f\n", offset_u, offset_v, offset_w);
    // Calibrate the current sense
}