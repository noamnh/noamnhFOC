#include "inverter.hpp"

 Inverter::Inverter() {
    // Constructor implementation
    ESP_LOGI("Inverter", "Inverter instance created.");
}
 Inverter::~Inverter() {
    // Destructor implementation
    // Clean up resources, if any
    for (int i = 0; i < 3; ++i) {
        if (operators[i]) {
            mcpwm_del_operator(operators[i]);
            operators[i] = nullptr;
        }
        if (comparators[i]) {
            mcpwm_del_comparator(comparators[i]);
            comparators[i] = nullptr;
        }
    }
    for (int i = 0; i < 6; ++i) {
        if (generators[i]) {
            mcpwm_del_generator(generators[i]);
            generators[i] = nullptr;
        }
    }
    if (timer) {
        mcpwm_del_timer(timer);
        timer = nullptr;
    }
    ESP_LOGI("Inverter", "Inverter instance destroyed.");
}
esp_err_t Inverter::on_init() {
    // Initialization logic
    ESP_LOGI("Inverter", "Inverter initialized.");
        esp_err_t ret = ESP_OK;

    ret = create_timer();
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to create MCPWM timer: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = create_operators();
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to create MCPWM operators: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = create_generators();

    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to create MCPWM generators: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mcpwm_timer_enable(timer);

    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to enable MCPWM timer");
        return ret;
    }
    
    ret = mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to start MCPWM timer");
        return ret;
    }

    // Set initial duty cycle to 0 for all phases
    ret = set_duty_cycle(0.0f, 0.0f, 0.0f);
    ESP_LOGI("Inverter", "MCPWM timer started successfully.");
    return ret; // Return success

}
esp_err_t Inverter::on_configure(inverter_config config) {
    config_ = config; 
    // Configuration logic
    ESP_LOGI("Inverter", "Inverter configured.");
    return ESP_OK; // Return success
}

esp_err_t Inverter::set_duty_cycle(float duty_a, float duty_b, float duty_c) {
    // Clamp duty cycles to valid range [0.0, 1.0]

    esp_err_t ret = ESP_OK;

    duty_a = fmax(0.0f, fmin(1.0f, duty_a));
    duty_b = fmax(0.0f, fmin(1.0f, duty_b));
    duty_c = fmax(0.0f, fmin(1.0f, duty_c));
    
    // Convert to timer ticks
    uint32_t compare_a = (uint32_t)(duty_a * PWM_RESOLUTION);
    uint32_t compare_b = (uint32_t)(duty_b * PWM_RESOLUTION);
    uint32_t compare_c = (uint32_t)(duty_c * PWM_RESOLUTION);
    
    // Set comparator values
    ret = mcpwm_comparator_set_compare_value(comparators[0], compare_a);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to set comparator A value: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = mcpwm_comparator_set_compare_value(comparators[1], compare_b);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to set comparator B value: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = mcpwm_comparator_set_compare_value(comparators[2], compare_c);
    if (ret != ESP_OK) {
        ESP_LOGE("Inverter", "Failed to set comparator C value: %s", esp_err_to_name(ret));
        return ret;
    }

    
    
    return ESP_OK; // Return success
}

esp_err_t Inverter::create_timer() {
    // Create the MCPWM timer
    ESP_LOGI("Inverter", "Creating MCPWM timer.");
    mcpwm_timer_config_t timer_config {};
    timer_config.group_id = 0; // MCPWM group ID
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT; // Default clock source
    timer_config.resolution_hz = PWM_RESOLUTION_HZ; // Resolution in Hz
    timer_config.period_ticks = PWM_RESOLUTION; // Period in ticks
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP; // Count mode

    // Implementation to create the timer
    return mcpwm_new_timer(&timer_config, &timer);; // Return success
}

esp_err_t Inverter::create_operators() {
    // Create the MCPWM operators for each phase
    ESP_LOGI("Inverter", "Creating MCPWM operators.");


            esp_err_t ret = ESP_OK;
        
        for (int i = 0; i < 3; i++) {
            // Create operator
            mcpwm_operator_config_t operator_config{};
            operator_config.group_id = 0; // MCPWM group ID

            ret = mcpwm_new_operator(&operator_config, &operators[i]);
            if (ret != ESP_OK) return ret;
            
            // Connect operator to timer
            ret = mcpwm_operator_connect_timer(operators[i], timer);
            if (ret != ESP_OK) return ret;
            
            // Create comparator
            mcpwm_comparator_config_t comparator_config = {
                .flags = {
                    .update_cmp_on_tez = true,
                },
            };
            ret = mcpwm_new_comparator(operators[i], &comparator_config, &comparators[i]);
            if (ret != ESP_OK) return ret;
        }

    return ESP_OK; // Return success
}

esp_err_t Inverter::create_generators() {
    ESP_LOGI("Inverter", "Creating MCPWM generators (corrected).");
    esp_err_t ret = ESP_OK;

    // Dead time config (shared for all phases)
    mcpwm_dead_time_config_t dt_config;
    dt_config.posedge_delay_ticks = (uint32_t)((float)DEAD_TIME_US * PWM_RESOLUTION_HZ / 1000000.0f);
    dt_config.negedge_delay_ticks = (uint32_t)((float)DEAD_TIME_US * PWM_RESOLUTION_HZ / 1000000.0f);
    dt_config.flags.invert_output = false; // No inversion for dead time

    // Optional: You can define inv_dt_config if you want different deadtime for low side
    mcpwm_dead_time_config_t inv_dt_config = dt_config; // For now just copy

    // Loop over PHASE A / B / C
    for (int phase = 0; phase < 3; ++phase) {
        ESP_LOGI("Inverter", "Configuring Phase %c generators.", 'A' + phase);

        // HIGH side generator
        mcpwm_generator_config_t gen_config_high;
        gen_config_high.gen_gpio_num = (phase == 0) ? config_.phase_a_high :
                                       (phase == 1) ? config_.phase_b_high :
                                                      config_.phase_c_high;

        ret = mcpwm_new_generator(operators[phase], &gen_config_high, &generators[phase * 2]);
        if (ret != ESP_OK) return ret;

        // LOW side generator
        mcpwm_generator_config_t gen_config_low;
        gen_config_low.gen_gpio_num = (phase == 0) ? config_.phase_a_low :
                                      (phase == 1) ? config_.phase_b_low :
                                                     config_.phase_c_low;

        ret = mcpwm_new_generator(operators[phase], &gen_config_low, &generators[phase * 2 + 1]);
        if (ret != ESP_OK) return ret;

        // Set HIGH side generator actions (UP + DOWN events)
        ret = mcpwm_generator_set_actions_on_compare_event(generators[phase * 2],
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[phase], MCPWM_GEN_ACTION_LOW),
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comparators[phase], MCPWM_GEN_ACTION_HIGH),
            MCPWM_GEN_COMPARE_EVENT_ACTION_END());
        if (ret != ESP_OK) return ret;

        // Apply Dead Time
        // Self dead time
        ret = mcpwm_generator_set_dead_time(generators[phase * 2], generators[phase * 2], &dt_config);
        if (ret != ESP_OK) return ret;

        // Cross dead time: high → low
        ret = mcpwm_generator_set_dead_time(generators[phase * 2], generators[phase * 2 + 1], &inv_dt_config);
        if (ret != ESP_OK) return ret;

        ESP_LOGI("Inverter", "Phase %c generators configured with dead time.", 'A' + phase);
    }

    return ESP_OK;
}