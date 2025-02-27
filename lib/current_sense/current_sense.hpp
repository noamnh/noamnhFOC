#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADC_WIDTH ADC_WIDTH_BIT_12   // 12-bit ADC resolution (0-4095)
#define ADC_ATTEN ADC_ATTEN_DB_11    // Attenuation for 0-3.9V range (safe for 3.3V input)
#define DEFAULT_VREF 1100            // Default reference voltage in mV

struct CurrentSenseConfig {
    int u_pin = 2;
    int v_pin = 3;
    int w_pin = 4;
    float shunt_resistance = 0.005;
    float gain = 200;
    
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
    void set_offsets(float offset_u, float offset_v, float offset_w) {
        offset_u_ = offset_u;
        offset_v_ = offset_v;
        offset_w_ = offset_w;
    }

    double get_offset_u() {
        return offset_u_;
    }

    double get_offset_v() {
        return offset_v_;
    }

    float getAveragedADC(int pin) {
        float sum = 0;
        for (int i = 0; i < 50; i++) {
            sum += adc1_get_raw((adc1_channel_t)pin);
        }
        return sum / 50;
    }

    private:
    CurrentSenseConfig config_;
    float volts_to_amps_ratio_;
    float offset_u_;
    float offset_v_;
    float offset_w_;
    float last_sum_ = 0;
};