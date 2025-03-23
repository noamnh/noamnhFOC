#include "current_sense.hpp"
#include <math.h>

CurrentSense::CurrentSense() : offset_u_(0), offset_v_(0), offset_w_(0), iu_(0), iv_(0), iw_(0) {}

void CurrentSense::on_configure(const CurrentSenseConfig& config) {
    config_ = config;
    volts_to_amps_ratio_ = 1 / (config_.shunt_resistance * config_.gain);
    printf("CurrentSense configured\n");
    printf("Shunt resistance: %f, Gain: %f\n", config_.shunt_resistance, config_.gain);
    printf("Volts to Amps ratio: %f\n", volts_to_amps_ratio_);
}

void CurrentSense::init_adc_dma() {
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = SAMPLE_BUFFER_SIZE,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &adc_handle_));

    adc_digi_pattern_config_t pattern[3] = {
        {.atten = ADC_ATTEN, .channel = config_.u_channel, .unit = ADC_UNIT_1, .bit_width = ADC_WIDTH},
        {.atten = ADC_ATTEN, .channel = config_.v_channel, .unit = ADC_UNIT_1, .bit_width = ADC_WIDTH},
        {.atten = ADC_ATTEN, .channel = config_.w_channel, .unit = ADC_UNIT_1, .bit_width = ADC_WIDTH},
    };

    adc_continuous_config_t adc_config;
    adc_config.sample_freq_hz = 20000;
    adc_config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    adc_config.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    adc_config.pattern_num = 3;
    adc_config.adc_pattern = pattern;

    ESP_ERROR_CHECK(adc_continuous_config(adc_handle_, &adc_config));
    ESP_ERROR_CHECK(adc_continuous_start(adc_handle_));
}

void CurrentSense::on_init() {
    init_adc_dma();
}

void CurrentSense::on_activate() {}
void CurrentSense::on_deactivate() {}


void CurrentSense::sample() {
    uint8_t result[SAMPLE_BUFFER_SIZE];
    uint32_t bytes_read = 0;
    int sum_u = 0, sum_v = 0, sum_w = 0;
    int count_u = 0, count_v = 0, count_w = 0;

    if (adc_continuous_read(adc_handle_, result, READ_LEN, &bytes_read, 0) == ESP_OK) {
        for (int i = 0; i < READ_LEN; i += sizeof(adc_digi_output_data_t)) {
            adc_digi_output_data_t* data = (adc_digi_output_data_t*)&result[i];
            if (data->type2.unit == ADC_UNIT_1) {
                int ch = data->type2.channel;
                int val = data->type2.data;
                if (ch == config_.u_channel) { sum_u += val; count_u++; }
                else if (ch == config_.v_channel) { sum_v += val; count_v++; }
                else if (ch == config_.w_channel) { sum_w += val; count_w++; }
            }
        }
    }

    constexpr float ADC_SCALE = 3.3f / 4095.0f;
    float vu = (count_u > 0) ? (sum_u / (float)count_u) * ADC_SCALE : 0.0f;
    float vv = (count_v > 0) ? (sum_v / (float)count_v) * ADC_SCALE : 0.0f;
    float vw = (count_w > 0) ? (sum_w / (float)count_w) * ADC_SCALE : 0.0f;

    float current_factor = 1.0f / (config_.shunt_resistance * config_.gain);
    iu_ = (vu - offset_u_) * current_factor;
    iv_ = (vv - offset_v_) * current_factor;
    iw_ = (vw - offset_w_) * current_factor;

     i_ = sqrt(iu_ * iu_ + iv_ * iv_ + iw_ * iw_);

}
void CurrentSense::calibrate() {
    offset_u_ = offset_v_ = offset_w_ = 0;
    int samples = 300;
    for (int i = 0; i < samples; ++i) {
        sample();
        offset_u_ += iu_;
        offset_v_ += iv_;
        offset_w_ += iw_;
        // vTaskDelay(pdMS_TO_TICKS(1));
    }
    offset_u_ /= samples;
    offset_v_ /= samples;
    offset_w_ /= samples;
    set_offsets(offset_u_, offset_v_, offset_w_);
    printf("Offsets calibrated: %f, %f, %f\n", offset_u_, offset_v_, offset_w_);
}

void CurrentSense::set_offsets(float offset_u, float offset_v, float offset_w) {
    offset_u_ = offset_u;
    offset_v_ = offset_v;
    offset_w_ = offset_w;
}

float CurrentSense::get_iu() { return iu_; }
float CurrentSense::get_iv() { return iv_; }
float CurrentSense::get_iw() { return iw_; }
float CurrentSense::get_i() { return i_; }
float CurrentSense::get_offset_u() { return offset_u_; }
float CurrentSense::get_offset_v() { return offset_v_; }
float CurrentSense::get_offset_w() { return offset_w_; }