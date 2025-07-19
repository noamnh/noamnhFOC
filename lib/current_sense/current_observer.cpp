#include "current_observer.hpp"
#include "esp_adc/adc_continuous.h"
#include <cstring>

using namespace current_sense;

CurrentObserver::CurrentObserver() {}

CurrentObserver::~CurrentObserver() {
    if (adc_handle_) {
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

esp_err_t CurrentObserver::on_configure(int adc_channel_ia, int adc_channel_ib, int adc_unit) {
    adc_channel_ia_ = adc_channel_ia;
    adc_channel_ib_ = adc_channel_ib;
    adc_unit_ = adc_unit;
    if (!adc_handle_) return ESP_ERR_INVALID_STATE;

    adc_continuous_config_t dig_config = {};
    // TODO: This should be config.. 
    dig_config.sample_freq_hz = 20000; // 20 kHz // This should be config.. 
    dig_config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    dig_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    dig_config.pattern_num = 2;

    adc_digi_pattern_config_t adc_pattern[2] = {};
    adc_pattern[0].atten = ADC_ATTEN_DB_11;
    adc_pattern[0].channel = adc_channel_ia_ & 0x7;
    adc_pattern[0].unit = adc_unit_;
    adc_pattern[0].bit_width = ADC_BITWIDTH_11;

    adc_pattern[1].atten = ADC_ATTEN_DB_11;
    adc_pattern[1].channel = adc_channel_ib_ & 0x7;
    adc_pattern[1].unit = adc_unit_;
    adc_pattern[1].bit_width = ADC_BITWIDTH_11;

    dig_config.adc_pattern = adc_pattern;

    return adc_continuous_config(adc_handle_, &dig_config);
}

esp_err_t CurrentObserver::on_activate() {
    if (!adc_handle_) return ESP_ERR_INVALID_STATE;
    return adc_continuous_start(adc_handle_);
}

esp_err_t CurrentObserver::on_deactivate() {
    if (!adc_handle_) return ESP_ERR_INVALID_STATE;
    return adc_continuous_stop(adc_handle_);
}

// --- Data Acquisition ---

// Internal storage for latest values
static float latest_ia = 0.0f;
static float latest_ib = 0.0f;

// Helper: parse DMA buffer and update latest values
void CurrentObserver::handle_dma_event() {
    uint8_t result[sizeof(dma_buffer_)];
    uint32_t length = 0;
    esp_err_t ret = adc_continuous_read(adc_handle_, result, sizeof(result), &length, 0);
    if (ret != ESP_OK || length == 0) return;

    // Parse all samples in the buffer
    for (uint32_t i = 0; i + sizeof(adc_digi_output_data_t) <= length; i += sizeof(adc_digi_output_data_t)) {
        adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
        int channel = p->type2.channel;
        int value = p->type2.data;
        if (channel == (adc_channel_ia_ & 0x7)) {
            latest_ia = (float)value;
        } else if (channel == (adc_channel_ib_ & 0x7)) {
            latest_ib = (float)value;
        }
    }
    data_ready_ = true;
}

// --- Calibration ---
esp_err_t CurrentObserver::on_calibrate() {
    // Simple calibration: average N samples for offset
    const int N = 100;
    float sum_ia = 0.0f, sum_ib = 0.0f;
    int count_ia = 0, count_ib = 0;
    for (int i = 0; i < N; ++i) {
        handle_dma_event();
        if (data_ready_) {
            sum_ia += latest_ia;
            sum_ib += latest_ib;
            count_ia++;
            count_ib++;
        }
        vTaskDelay(1); // Small delay between samples
    }
    if (count_ia > 0) offset_ia_ = sum_ia / count_ia;
    if (count_ib > 0) offset_ib_ = sum_ib / count_ib;
    return ESP_OK;
}

// --- Get Currents ---
bool CurrentObserver::get_currents(float& ia, float& ib) {
    handle_dma_event();
    if (!data_ready_) return false;
    // Convert ADC value to Amps: (raw - offset) * scale
    ia = (latest_ia - offset_ia_) * scale_ia_;
    ib = (latest_ib - offset_ib_) * scale_ib_;
    data_ready_ = false;
    return true;
} 