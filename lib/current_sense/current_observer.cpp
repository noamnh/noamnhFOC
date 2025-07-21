#include "current_observer.hpp"
#include "esp_adc/adc_continuous.h"
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

using namespace current_sense;

CurrentObserver::CurrentObserver() {}

CurrentObserver::~CurrentObserver() {
    if (adc_handle_ != nullptr) {
        adc_continuous_deinit(adc_handle_);
        adc_handle_ = nullptr;
    }
}

esp_err_t CurrentObserver::on_init() {
    adc_continuous_handle_cfg_t adc_config = {};
    adc_config.max_store_buf_size = sizeof(dma_buffer_);
    adc_config.conv_frame_size = sizeof(dma_buffer_);
    return adc_continuous_new_handle(&adc_config, &adc_handle_);
}

esp_err_t CurrentObserver::on_configure(int gpio_ia, int gpio_ib, int gpio_ic, Config config) {
    adc_unit_t unit_ia, unit_ib, unit_ic;
    adc_channel_t ch_ia, ch_ib, ch_ic;
    config_ = config;

    // Convert GPIO to ADC unit and channel
    if (adc_continuous_io_to_channel(gpio_ia, &unit_ia, &ch_ia) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (adc_continuous_io_to_channel(gpio_ib, &unit_ib, &ch_ib) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (gpio_ic >= 0 && adc_continuous_io_to_channel(gpio_ic, &unit_ic, &ch_ic) != ESP_OK) return ESP_ERR_INVALID_ARG;

    // For now, require all on ADC_UNIT_1 (ESP32-S3 limitation)
    if (unit_ia != ADC_UNIT_1 || unit_ib != ADC_UNIT_1 || (gpio_ic >= 0 && unit_ic != ADC_UNIT_1)) return ESP_ERR_INVALID_ARG;

    adc_channel_ia_ = ch_ia;
    adc_channel_ib_ = ch_ib;
    adc_channel_ic_ = (gpio_ic >= 0) ? ch_ic : -1;
    adc_unit_ = ADC_UNIT_1;
    if (adc_handle_ == nullptr) return ESP_ERR_INVALID_STATE;

    adc_continuous_config_t dig_config = {};
    dig_config.sample_freq_hz = 20000; // 20 kHz
    dig_config.conv_mode = ADC_CONV_SINGLE_UNIT_1; // Use single ADC unit
    dig_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    dig_config.pattern_num = 3; // Always use 3 channels

    adc_digi_pattern_config_t adc_pattern[3] = {};
    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = adc_channel_ia_;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;

    adc_pattern[1].atten = ADC_ATTEN_DB_12;
    adc_pattern[1].channel = adc_channel_ib_;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = ADC_BITWIDTH_12;

    adc_pattern[2].atten = ADC_ATTEN_DB_12;
    adc_pattern[2].channel = adc_channel_ic_;
    adc_pattern[2].unit = ADC_UNIT_1;
    adc_pattern[2].bit_width = ADC_BITWIDTH_12;

    dig_config.adc_pattern = adc_pattern;

    // Pre-calculate scale factor for performance
    // Scale factor = ADC_VREF / (shunt_resistor * amplifier_gain * ADC_MAX)
    current_scale_factor_ = ADC_VREF / (config.shunt_resistor * config.amplifier_gain * ADC_MAX);
    lpf_gain_ = config.lpf_gain;

    return adc_continuous_config(adc_handle_, &dig_config);
}

esp_err_t CurrentObserver::on_activate() {
    if (adc_handle_ == nullptr) return ESP_ERR_INVALID_STATE;
    return adc_continuous_start(adc_handle_);
}

esp_err_t CurrentObserver::on_deactivate() {
    if (adc_handle_ == nullptr) return ESP_ERR_INVALID_STATE;
    return adc_continuous_stop(adc_handle_);
}

// --- Data Acquisition ---

// Internal storage for latest values
static float latest_ia = 0.0f;
static float latest_ib = 0.0f;
static float latest_ic = 0.0f;

// Helper: parse DMA buffer and update latest values
void CurrentObserver::handle_dma_event() {
    uint32_t length = 0;
    esp_err_t ret = adc_continuous_read(adc_handle_, (uint8_t*)dma_buffer_, sizeof(dma_buffer_), &length, 0);
    if (ret != ESP_OK || length == 0) return;

    // Parse all samples in the buffer - optimized for 3 channels
    uint32_t num_samples = length / sizeof(adc_digi_output_data_t);
    adc_digi_output_data_t *p = (adc_digi_output_data_t *)dma_buffer_;
    
    for (uint32_t i = 0; i < num_samples; i++) {
        int channel = p[i].type2.channel;
        int value = p[i].type2.data;
        
        // Direct assignment - no float conversion overhead
        if (channel == adc_channel_ia_) {
            latest_ia = (float)value;
        } else if (channel == adc_channel_ib_) {
            latest_ib = (float)value;
        } else if (channel == adc_channel_ic_) {
            latest_ic = (float)value;
        }
    }
    data_ready_ = true;
    // ESP_LOGI("CurrentObserver", "Data ready: IA=%.2f, IB=%.2f, IC=%.2f", latest_ia, latest_ib, latest_ic);
}

// --- Calibration ---
esp_err_t CurrentObserver::on_calibrate() {
    // Simple calibration: average N samples for offset
    const int N = 100;
    float sum_ia = 0.0f, sum_ib = 0.0f, sum_ic = 0.0f;
    int count_ia = 0, count_ib = 0, count_ic = 0;
    for (int i = 0; i < N; ++i) {
        handle_dma_event();
        if (data_ready_) {
            sum_ia += latest_ia;
            sum_ib += latest_ib;
            sum_ic += latest_ic;
            count_ia++;
            count_ib++;
            count_ic++;
        }
        vTaskDelay(1); // Small delay between samples
    }
    if (count_ia > 0) offset_ia_ = sum_ia / count_ia;
    if (count_ib > 0) offset_ib_ = sum_ib / count_ib;
    if (count_ic > 0) offset_ic_ = sum_ic / count_ic;

    // log the offset values
    ESP_LOGI("CurrentObserver", "Calibration completed: IA=%.3f, IB=%.3f, IC=%.3f", offset_ia_, offset_ib_, offset_ic_);

    return ESP_OK;
}


esp_err_t CurrentObserver::on_calibrate_ema() {
    // Parameters for EMA
    const int N = 100; // Number of samples (for how long to run)
    const float tau = 0.1f; // Time constant in seconds (tune as needed)
    const float update_rate_hz = 1000.0f; // How often you sample (Hz)
    const float D = 1.0f - expf(-1.0f / (tau * update_rate_hz));

    float offset_ia = 0.0f, offset_ib = 0.0f, offset_ic = 0.0f;
    bool first = true;

    for (int i = 0; i < N; ++i) {
        handle_dma_event();
        if (data_ready_) {
            if (first) {
                // Initialize with first sample
                offset_ia = latest_ia;
                offset_ib = latest_ib;
                offset_ic = latest_ic;
                first = false;
            } else {
                // EMA update
                offset_ia += D * (latest_ia - offset_ia);
                offset_ib += D * (latest_ib - offset_ib);
                offset_ic += D * (latest_ic - offset_ic);
            }
        }
        vTaskDelay(1); // Small delay between samples
    }

    offset_ia_ = offset_ia;
    offset_ib_ = offset_ib;
    offset_ic_ = offset_ic;

    ESP_LOGI("CurrentObserver", "EMA Calibration completed: IA=%.3f, IB=%.3f, IC=%.3f", offset_ia_, offset_ib_, offset_ic_);

    return ESP_OK;
}


// --- Get Currents ---
bool CurrentObserver::get_currents(float& ia, float& ib, float& ic) {
    if (!data_ready_) return false;
    

    
    // Optimized current calculation using pre-calculated scale factor
    // Single multiplication: (ADC_raw - offset) * scale_factor
    const float raw_ia = (latest_ia - offset_ia_) * current_scale_factor_;
    const float raw_ib = (latest_ib - offset_ib_) * current_scale_factor_;
    const float raw_ic = (latest_ic - offset_ic_) * current_scale_factor_;

    // if (debug_counter % 1000 == 0) {
    //     ESP_LOGI("CurrentObserver", "Currents: IA=%.4f, IB=%.4f, IC=%.4f A", raw_ia, raw_ib, raw_ic);
    // }
    
    // Apply LPF with 3-phase constraint using pre-calculated gain
    // We are using 3 phase system so the sum is zero
    // We can filter between actual phase currents by subtracting the sum of the other two phases
    ia = ((1.0f - lpf_gain_) * raw_ia) - (lpf_gain_ * (raw_ib+raw_ic));
    ib = ((1.0f - lpf_gain_) * raw_ib) - (lpf_gain_ * (raw_ia+raw_ic));
    ic = ((1.0f - lpf_gain_) * raw_ic) - (lpf_gain_ * (raw_ia+raw_ib));
    
    // Return the calculated currents
    // ia = raw_ia;
    // ib = raw_ib;
    // ic = raw_ic;
    
    data_ready_ = false;
    return true;
}

 