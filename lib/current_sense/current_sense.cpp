#include "current_sense.hpp"


CurrentSense::CurrentSense() {

}
esp_err_t CurrentSense::on_configure() {


}

esp_err_t CurrentSense::on_init() {

    adc_continuous_handle_cfg_t adc_config;
    adc_config.max_store_buf_size = 2048;
    adc_config.conv_frame_size = EXAMPLE_READ_LEN;

    esp_err_t ret = adc_continuous_new_handle(&adc_config, &adc_handler_);
    if (ret != ESP_OK) {
        return ret;
    }
    adc_continuous_config_t dig_config;
    dig_config.sample_freq_hz = 20000;
    dig_config.conv_mode = EXAMPLE_ADC_CONV_MODE;
    dig_config.format = EXAMPLE_ADC_OUTPUT_TYPE;
    dig_config.pattern_num = 3; // Number of ADC channels to use

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};

    for(int i = 0; i < dig_config.pattern_num; i++) {
        adc_pattern[i].atten = EXAMPLE_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = EXAMPLE_ADC_UNIT;
        adc_pattern[i].bit_width = EXAMPLE_ADC_BIT_WIDTH;
    }
    dig_config.adc_pattern = adc_pattern;
     ret = adc_continuous_config(adc_handler_, &dig_config);

    return ret;
}