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

    float raw_u = getAveragedADC(config_.u_pin);
    float raw_v = getAveragedADC(config_.v_pin);
    float raw_w = getAveragedADC(config_.w_pin);
    
    // Convert raw ADC values to voltage (in volts)
    float voltage_u = (raw_u * 3.3) / 4095;  // Scale to 3.3V
    float voltage_v = (raw_v * 3.3) / 4095;
    float voltage_w = (raw_w * 3.3) / 4095;
    
    // Subtract offset (1.65V reference)
    voltage_u -= offset_u_;
    voltage_v -= offset_v_;
    voltage_w -= offset_w_;
    
    // Convert to current (without gain)
    float current_u = voltage_u / (config_.gain*config_.shunt_resistance);
    float current_v = voltage_v / (config_.gain*config_.shunt_resistance);
    float current_w = voltage_w / (config_.gain*config_.shunt_resistance);

    
    // Compute total current sum (absolute values)
    float sum = current_u + current_v + current_w;
    
    // Estimate the gain
    // printf("Sum of Currents: %.4f A (Should be close to 0)\n", sum);
    

}

void CurrentSense::calibrate() {

    float offset_u = 0;
    float offset_v = 0;
    float offset_w = 0;

    for (int i = 0; i < 100; i++) {
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
        vTaskDelay(1 / portTICK_PERIOD_MS);
        
    }
    offset_u /= 100;
    offset_v /= 100;
    offset_w /= 100;
    set_offsets(offset_u/1000, offset_v/1000, offset_w/1000);
    // printf("CurrentSense Calibration - U: %f mV, V: %f mV\n", offset_u_, offset_v_);
    // Calibrate the current sense
}