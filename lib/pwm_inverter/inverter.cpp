#include "inverter.hpp"
#include "esp_err.h"


Inverter::Inverter() {
    // Constructor
}

void Inverter::on_configure(InverterConfig config) {
    // Configure the inverter
    config_ = config;
            gpio_config_t io_conf_1;
        io_conf_1.intr_type = GPIO_INTR_DISABLE;
        io_conf_1.mode = GPIO_MODE_OUTPUT;
        io_conf_1.pin_bit_mask = (1ULL << config_.enable_pin);
        io_conf_1.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf_1.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf_1);
        gpio_set_level(static_cast<gpio_num_t>(config_.enable_pin), 0); 

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE, // Changed to LOW_SPEED_MODE
        .duty_resolution = LEDC_TIMER_10_BIT, // 10-bit resolution
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000, // 20 kHz switching frequency
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
        
        int gpio_pins[] = {config_.u_pin, config_.v_pin, config_.w_pin};

            for (int i = 0; i < 3; ++i) {
        ledc_channel_config_t channel_conf = {
            .gpio_num = static_cast<gpio_num_t>(gpio_pins[i]),
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = static_cast<ledc_channel_t>(LEDC_CHANNEL_0 + i),
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel_conf);
    }

}

void Inverter::on_init() {
    // Initialize the inverter
    // We are using one timer - the timer is used to generate the PWM signal
    // Each pwm has a duty cycle and a frequency
    // multiple channels can share the same timer
    // because our channels share the same timer, they will have the same frequency


}


void Inverter::on_activate() {
    gpio_set_level(config_.enable_pin, 1);

}


void Inverter::on_deactivate() {
    gpio_set_level(config_.enable_pin, 0);

}

void Inverter::set_uvw(float u, float v, float w) {
    auto set_pwm = [](int channel, float voltage, float max_voltage) {
        float duty_cycle = (voltage / max_voltage + 0.5f) * 1023.0f; // 10-bit resolution
        duty_cycle = clamp(duty_cycle, 0.0f, 1023.0f);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel), static_cast<uint32_t>(duty_cycle));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel));
    };
    set_pwm(LEDC_CHANNEL_0, u, config_.dc_voltage_ps);
    set_pwm(LEDC_CHANNEL_1, v, config_.dc_voltage_ps);
    set_pwm(LEDC_CHANNEL_2, w, config_.dc_voltage_ps);
}

void Inverter::on_update(){

}