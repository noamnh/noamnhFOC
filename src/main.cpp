
#include <cmath>
#include <iostream>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Controller.hpp"
#include "esp_timer.h"


FieldOrientedControl foc;
Inverter inverter;
FOCController controller;
Motor motor;
EncoderI2C encoder(6, 7, 0x36, 800000);
CurrentSense current_sense;
InverterConfig inverter_config;
MotorConfig motor_config;
CurrentSenseConfig current_sense_config;

constexpr float PI = 3.14159265358979f;
constexpr float V_SUPPLY = 12.0f;
constexpr int POLE_PAIRS = 7;
constexpr int GPIO_U = 8;
constexpr int GPIO_V = 9;
constexpr int GPIO_W = 10;
constexpr int GPIO_ENABLE = 5;

constexpr int ADC_U_PIN = 2;
constexpr int ADC_V_PIN = 3;
constexpr int ADC_W_PIN = 4;


void IRAM_ATTR focControllerCallback(void* arg) {
    controller.run();
    
}




extern "C" void app_main() {
        vTaskDelay(pdMS_TO_TICKS(1000)); // 10 ms delay
    inverter_config.u_pin = GPIO_U;
    inverter_config.v_pin = GPIO_V;
    inverter_config.w_pin = GPIO_W;
    inverter_config.enable_pin = GPIO_NUM_5;
    inverter_config.dc_voltage_ps = V_SUPPLY;
    inverter.on_configure(inverter_config);
    motor_config.pole_pairs = POLE_PAIRS;
    motor_config.max_voltage = V_SUPPLY;
    motor.on_configure(motor_config);
    current_sense_config.u_pin = ADC_U_PIN;
    current_sense_config.v_pin = ADC_V_PIN;
    current_sense_config.w_pin = ADC_W_PIN;
    current_sense.on_configure(current_sense_config);
    controller.on_init(inverter, encoder, motor, current_sense);

    vTaskDelay(3000 / portTICK_PERIOD_MS);
    // current_sense.calibrate();


    controller.align();

    controller.on_activate();
      const esp_timer_create_args_t timer_args = {
        .callback = &focControllerCallback,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "foc_timer"
    };

    esp_timer_handle_t foc_timer;
    esp_err_t ret = esp_timer_create(&timer_args, &foc_timer);
    if (ret != ESP_OK) {
        // Handle error
    }

    // Start the timer with a period of 100 microseconds (10 kHz)
    ret = esp_timer_start_periodic(foc_timer, 100);
    if (ret != ESP_OK) {
        // Handle error
    }

}

