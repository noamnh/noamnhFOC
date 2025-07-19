#pragma once
#include <stdint.h>
#include "esp_err.h"

// Forward declarations for ESP-IDF ADC types
struct adc_continuous_handle_t;

namespace current_sense {

class CurrentObserver {
public:
    CurrentObserver();
    ~CurrentObserver();

    // Allocate ADC handle and basic setup
    esp_err_t on_init();
    // Configure channels, patterns, calibration, etc.
    esp_err_t on_configure(int adc_channel_ia, int adc_channel_ib, int adc_unit = 1);
    // Start ADC sampling (DMA)
    esp_err_t on_activate();
    // Stop ADC sampling
    esp_err_t on_deactivate();

    // Get latest current values (in amps, after calibration)
    bool get_currents(float& ia, float& ib);

    // (Optional) Calibrate offsets
    esp_err_t on_calibrate();

private:
    adc_continuous_handle_t* adc_handle_ = nullptr;
    int adc_channel_ia_ = -1;
    int adc_channel_ib_ = -1;
    int adc_unit_ = 1;

    // DMA buffer and state
    uint16_t dma_buffer_[16]; // Example size
    volatile bool data_ready_ = false;

    // Calibration data
    float offset_ia_ = 0.0f;
    float offset_ib_ = 0.0f;
    float scale_ia_ = 1.0f;
    float scale_ib_ = 1.0f;

    // Internal methods
    void handle_dma_event();
};

} // namespace current_sense 