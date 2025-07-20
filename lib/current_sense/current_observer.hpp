#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "math.h"
// Include ESP-IDF ADC types
#include "esp_adc/adc_continuous.h"

// ADC constants - using defines to save heap memory
#define ADC_VREF 3.3f
#define ADC_MAX 4095.0f

// Fixed-point arithmetic defines for faster calculations
#define FIXED_POINT_SHIFT 16
#define FIXED_POINT_SCALE (1 << FIXED_POINT_SHIFT)  // 65536

namespace current_sense {

struct Config{
    float lpf_gain = 0.3f;
    float shunt_resistor = 0.005f;
    float amplifier_gain = 1.0f;
};

class CurrentObserver {
public:
    CurrentObserver();
    ~CurrentObserver();

    // Allocate ADC handle and basic setup
    esp_err_t on_init();
    // Configure using GPIO numbers (not ADC channels)
    esp_err_t on_configure(int gpio_ia, int gpio_ib, int gpio_ic = -1, Config config = Config());
    // Start ADC sampling (DMA)
    esp_err_t on_activate();
    // Stop ADC sampling
    esp_err_t on_deactivate();

    // Get latest current values (in amps, after calibration)
    bool get_currents(float& ia, float& ib, float& ic);

    // (Optional) Calibrate offsets
    esp_err_t on_calibrate();
    esp_err_t on_calibrate_ema();
    float  gain_ = 0.0f;
    int gain_count_ = 0;
    // float expected_current_ = 0.0f; // Expected current from power supply

private:
    adc_continuous_handle_t adc_handle_ = nullptr;
    int adc_channel_ia_ = -1;
    int adc_channel_ib_ = -1;
    int adc_channel_ic_ = -1;
    int adc_unit_ = 1;

    // DMA buffer and state
    uint16_t dma_buffer_[64]; // Increased for 3 channels at 20kHz - 3.2ms latency
    volatile bool data_ready_ = false;

    // Calibration data
    float offset_ia_ = 0.0f;
    float offset_ib_ = 0.0f;
    float offset_ic_ = 0.0f;

    // Pre-calculated constants for performance
    float current_scale_factor_ = 0.0f;  // 1.0f / (shunt_resistor * amplifier_gain * ADC_MAX / ADC_VREF)
    float lpf_gain_ = 0.0f;
    
    // Fixed-point scale factor for integer calculations
    uint32_t current_scale_factor_int_ = 0;  // (ADC_VREF * 65536) / (shunt * gain * ADC_MAX)

    Config config_;

    // Internal methods
    void handle_dma_event();
};

} // namespace current_sense 