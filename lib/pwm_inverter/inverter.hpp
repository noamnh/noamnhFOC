

#include "stdio.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "utils.hpp"

#define LEDC_FREQUENCY_HZ       15000
#define LEDC_RESOLUTION_STEPS   255.0
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define PWM_DUTY_RESOLUTION     (1 << LEDC_DUTY_RES) - 1


struct InverterConfig{
    gpio_num_t  enable_pin = GPIO_NUM_NC;
    uint8_t u_pin;
    uint8_t v_pin;
    uint8_t w_pin;
    float dc_voltage_ps = 12.0;
    float voltage_limit = 6.0;
};

struct InverterCmd{
    float u;
    float v;
    float w;
};

class Inverter {
    public:
    Inverter();
    void on_configure(InverterConfig config);
    void on_init();
    void on_activate();
    void on_deactivate();

    void on_update();
    void set_uvw(float u, float v, float w);
    InverterConfig get_config(){return config_;}
    
    private:
    InverterConfig config_;
    InverterCmd cmd_;


};