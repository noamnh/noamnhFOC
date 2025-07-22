#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#define UH 1
#define UL 2
#define VH 42
#define VL 41
#define WH 40
#define WL 39
#define ENABLE 38

#define PWM_FREQUENCY 20000
#define PWM_PERIOD_SEC 1.0f / PWM_FREQUENCY
#define MCPWM_PERIOD 1000
#define MCPWM_TIMER_RESOLUTION_HZ (PWM_FREQUENCY * MCPWM_PERIOD)
#define TICK_DURATION_NS        (1000000000 / MCPWM_TIMER_RESOLUTION_HZ)       // 50 ns
#define ADC_SAMPLE_OFFSET_US    6                      // Delay from true center in microseconds
#define ADC_SAMPLE_OFFSET_TICKS ((ADC_SAMPLE_OFFSET_US * 1000) / TICK_DURATION_NS)  // 6 us = 120 ticks
#define MCPWM_ADC_COMPARE_TICKS (MCPWM_PERIOD_TICKS / 2 + ADC_SAMPLE_OFFSET_TICKS)  // Midpoint + offset

typedef struct inverter_config_t {
    mcpwm_timer_config_t timer_config;          // pwm timer and timing config
    mcpwm_operator_config_t operator_config;    // mcpwm operator config
    mcpwm_comparator_config_t compare_config;   // mcpwm comparator config
    int gen_gpios[3][2];
    mcpwm_dead_time_config_t dt_config;         // dead time config for positive pwm output
    mcpwm_dead_time_config_t inv_dt_config;     // dead time config for negative pwm output
} inverter_config_t;


typedef struct mcpwm_handler_t {
    mcpwm_timer_handle_t timer;                 // MCPWM timer handle
    mcpwm_oper_handle_t operators[3];
    mcpwm_cmpr_handle_t comparators[3];
    mcpwm_gen_handle_t  generators[3][2];
    mcpwm_cmpr_handle_t adc_mid_comparator;
} mcpwm_handler_t;

namespace inverter{
class Inverter {
public:
Inverter();
~Inverter();
    esp_err_t on_init(mcpwm_comparator_event_callbacks_t* event = nullptr, void* ctx = nullptr);
    esp_err_t on_activate();
    esp_err_t on_deactivate();
    esp_err_t set_duty_cycle(float duty_a, float duty_b, float duty_c);
    esp_err_t set_inverter_callback(mcpwm_timer_event_callbacks_t *event, void *cb);

    inverter_config_t config_; // Configuration for the inverter
    mcpwm_handler_t mcpwm_handler_; // MCPWM handler containing all handles

};

}