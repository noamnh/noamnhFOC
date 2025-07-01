#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#define UH 1
#define UL 2
#define VH 3
#define VL 4
#define WH 7
#define WL 8
#define ENABLE 9

#define MCPWM_TIMER_RESOLUTION_HZ 20000000 // 10 MHz timer resolution 1 tick = 0.1µs
#define MCPWM_PERIOD 1000

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
} mcpwm_handler_t;

namespace inverter{
class Inverter {
public:
Inverter();
~Inverter();
    esp_err_t on_init();
    esp_err_t on_activate();
    esp_err_t on_deactivate();
    esp_err_t set_duty_cycle(float duty_a, float duty_b, float duty_c);
    esp_err_t set_inverter_callback(mcpwm_timer_event_callbacks_t *event, void *cb);

    inverter_config_t config_; // Configuration for the inverter
    mcpwm_handler_t mcpwm_handler_; // MCPWM handler containing all handles

};

}