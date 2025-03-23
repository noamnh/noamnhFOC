
#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <array>
#include <math.h>

#define ADC_WIDTH ADC_WIDTH_BIT_12   // 12-bit ADC resolution (0-4095)
#define ADC_ATTEN ADC_ATTEN_DB_12    // Attenuation for 0-3.9V range (safe for 3.3V input)
#define DEFAULT_VREF 1100            // Default reference voltage in mV
constexpr size_t SAMPLE_BUFFER_SIZE = 1024;
constexpr size_t READ_LEN = 256;


struct CurrentSenseConfig {
    adc_channel_t u_channel = ADC_CHANNEL_2;
    adc_channel_t v_channel = ADC_CHANNEL_3;
    adc_channel_t w_channel = ADC_CHANNEL_4;
    float shunt_resistance = 0.005;
    float gain = 2150;
    float bias_u = 1.65f;
    float bias_v = 1.65f;
    float bias_w = 1.65f;
};

class CurrentSense {
    public:
        CurrentSense();
        void on_configure(const CurrentSenseConfig& config);
        void on_init();
        void on_activate();
        void on_deactivate();
        void sample();
        void calibrate();
        void set_offsets(float offset_u, float offset_v, float offset_w);
        float get_offset_u();
        float get_offset_v();
        float get_offset_w();
        float get_iu();
        float get_iv();
        float get_iw();
        float get_i();

    
    private:
        CurrentSenseConfig config_;
        float volts_to_amps_ratio_;
        float offset_u_ = 0;
        float offset_v_ = 0;
        float offset_w_ = 0;
         float iu_ = 0;
         float iv_ = 0;
         float iw_ = 0;
         float i_ = 0;
        adc_continuous_handle_t adc_handle_ = nullptr;
        void init_adc_dma();
    };