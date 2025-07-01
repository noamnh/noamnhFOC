#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"





#define EXAMPLE_ADC_UNIT                    ADC_UNIT_1
#define EXAMPLE_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define EXAMPLE_ADC_ATTEN                   ADC_ATTEN_DB_11
#define EXAMPLE_ADC_BIT_WIDTH               ADC_BITWIDTH_11
#define EXAMPLE_ADC_OUTPUT_TYPE             ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define EXAMPLE_ADC_GET_CHANNEL(p_data)     ((p_data)->type2.channel)
#define EXAMPLE_ADC_GET_DATA(p_data)        ((p_data)->type2.data)
#define EXAMPLE_READ_LEN



class CurrentSense {
    public:
        CurrentSense();
        esp_err_t on_configure();
        esp_err_t on_init();

        private:
    adc_continuous_handle_t adc_handler_ = nullptr;
    adc_channel_t channel[3] = {ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4}; // U, V, W channels
};


