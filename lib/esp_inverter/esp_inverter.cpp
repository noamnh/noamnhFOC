#include "esp_inverter.hpp"
#include "freertos/FreeRTOS.h"
#include "utils.h"
using namespace inverter;

template <typename T>
T clamp(T value, T min_val, T max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}


Inverter::Inverter() {
    // Constructor implementation
    ESP_LOGI("Inverter", "ESP Inverter instance created.");
}
Inverter::~Inverter() {

}

esp_err_t Inverter::on_init() {

    esp_err_t ret;

    config_.timer_config.group_id = 0; // MCPWM group ID
    config_.timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT; // Default clock source
    config_.timer_config.resolution_hz = MCPWM_TIMER_RESOLUTION_HZ; // Timer resolution
    config_.timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN; // Count mode
    config_.timer_config.period_ticks = MCPWM_PERIOD; // Period in ticks

    config_.operator_config.group_id = 0; // MCPWM group ID

    config_.compare_config.flags.update_cmp_on_tez = true; // Update comparator on timer zero event
    
    config_.gen_gpios[0][0] = UH;
    config_.gen_gpios[0][1] = UL;
    config_.gen_gpios[1][0] = VH;
    config_.gen_gpios[1][1] = VL;
    config_.gen_gpios[2][0] = WH;
    config_.gen_gpios[2][1] = WL;


    config_.dt_config.posedge_delay_ticks = 5; // 0.5µs dead time for high side

    config_.inv_dt_config.negedge_delay_ticks = 5; // 0.5µs dead time for low side
    config_.inv_dt_config.flags.invert_output = true; // Invert output for low side

    ret = mcpwm_new_timer(&config_.timer_config, &mcpwm_handler_.timer);
    if (ret != ESP_OK || mcpwm_handler_.timer == nullptr) {
        ESP_LOGE("Inverter", "Failed to create MCPWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("Inverter", "MCPWM timer created successfully.");
ESP_LOGI("Inverter", "Timer pointer: %p", mcpwm_handler_.timer);

     for (int i = 0; i < 3; i++) {
        mcpwm_new_operator(&config_.operator_config, &mcpwm_handler_.operators[i]);
        ret = mcpwm_operator_connect_timer(mcpwm_handler_.operators[i], mcpwm_handler_.timer);
        ESP_LOGI("Inverter", "Connected operator %d to timer, ret=%s", i, esp_err_to_name(ret));

    }

    for (int i = 0; i < 3; i++) {
        mcpwm_new_comparator(mcpwm_handler_.operators[i], &config_.compare_config, &mcpwm_handler_.comparators[i]);
        mcpwm_comparator_set_compare_value(mcpwm_handler_.comparators[i], 0);
    }

    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 2; j++) {
            mcpwm_generator_config_t gen_config;
            gen_config.gen_gpio_num = config_.gen_gpios[i][j];
            mcpwm_new_generator(mcpwm_handler_.operators[i], &gen_config, &mcpwm_handler_.generators[i][j]);
        }

    }

    for(int i = 0; i < 3; i++) {
        mcpwm_generator_set_actions_on_compare_event(mcpwm_handler_.generators[i][0],
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_handler_.comparators[i], MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, mcpwm_handler_.comparators[i], MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END());

    }

    for (int i = 0; i < 3; i++) {
        mcpwm_generator_set_dead_time(mcpwm_handler_.generators[i][0], mcpwm_handler_.generators[i][0], &config_.dt_config);
        mcpwm_generator_set_dead_time(mcpwm_handler_.generators[i][0], mcpwm_handler_.generators[i][1], &config_.inv_dt_config);
    }

        gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
    // put gpio on 0
    gpio_set_level(GPIO_NUM_9, 0);

    ESP_LOGI("Inverter", "MCPWM operators, comparators and generators created successfully.");
    return ESP_OK;
    }

    esp_err_t Inverter::on_activate() {
        if (!mcpwm_handler_.timer) {
        ESP_LOGE("Inverter", "Timer handle is null");
        return ESP_FAIL;
    }

    ESP_LOGI("Inverter", "Enabling MCPWM timer...");
    
    esp_err_t ret = mcpwm_timer_enable(mcpwm_handler_.timer);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to enable MCPWM timer: %s", esp_err_to_name(ret));
        return ret;
    }


    ret = mcpwm_timer_start_stop(mcpwm_handler_.timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to start MCPWM timer: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = set_duty_cycle(0.0f, 0.0f, 0.0f);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to set duty cycle: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_level(GPIO_NUM_9, 1); // enable inverter
    ESP_LOGI("Inverter", "MCPWM timer started successfully.");
    return ESP_OK;
}

esp_err_t Inverter::on_deactivate() {
    esp_err_t ret = mcpwm_timer_start_stop(mcpwm_handler_.timer, MCPWM_TIMER_STOP_EMPTY);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to stop MCPWM timer: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI("Inverter", "MCPWM timer stopped successfully.");
    return ESP_OK;
}

esp_err_t Inverter::set_duty_cycle(float duty_a, float duty_b, float duty_c) {
    esp_err_t ret = ESP_OK;
    

        // Convert from [0.0, 1.0] float to timer ticks
    int32_t cmp_a = (int32_t)(duty_a * config_.timer_config.period_ticks/2);
    int32_t cmp_b = (int32_t)(duty_b * config_.timer_config.period_ticks/2);
    int32_t cmp_c = (int32_t)(duty_c * config_.timer_config.period_ticks/2);

    // Clamp to valid range just in case
    cmp_a = clamp(cmp_a,0,config_.timer_config.period_ticks/2);
    cmp_b = clamp(cmp_b,0,config_.timer_config.period_ticks/2);
    cmp_c = clamp(cmp_c,0,config_.timer_config.period_ticks/2);

    // log cmp_a b c
    // ESP_LOGI("Inverter","a: %ld, b: %ld, c: %ld", cmp_a, cmp_b, cmp_c);
    // i need to convert the duty of float to the units of the mcpwm right ? 
    // Set duty cycle for each phase
    ret = mcpwm_comparator_set_compare_value(mcpwm_handler_.comparators[0], cmp_a);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to set duty cycle for Phase A: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = mcpwm_comparator_set_compare_value(mcpwm_handler_.comparators[1], cmp_b);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to set duty cycle for Phase B: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = mcpwm_comparator_set_compare_value(mcpwm_handler_.comparators[2], cmp_c);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to set duty cycle for Phase C: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t Inverter::set_inverter_callback(mcpwm_timer_event_callbacks_t *event, void *cb) {
    esp_err_t ret = mcpwm_timer_register_event_callbacks(mcpwm_handler_.timer, event, cb);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to set inverter callback: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI("Inverter", "Inverter callback set successfully.");
    return ESP_OK;
}