// // #include "freertos/FreeRTOS.h"
// // #include "freertos/task.h"
// // #include "esp_system.h" 
// // #include "esp_log.h"
// // #include "encoders.hpp"
// // #include "Controller.hpp"
// // #include "inverter.hpp"
// // #include "motor.hpp"
// // #include <stdio.h>

// // EncoderI2C encoder(6, 7, 0x36, 800000);
// // Motor motor;
// // Inverter inverter;
// // FOCController controller;

// // MotorConfig motor_config;
// // InverterConfig inverter_config;

// // void task_read_encoder(void *pvParameter)
// // {
// //     while (1) {
// //         encoder.read();
// //         vTaskDelay(1 / portTICK_PERIOD_MS); // Add a delay to prevent CPU starvation
// //     }
// // }

// // void print_encoder_angle(void *pvParameter)
// // {
// //     while (1) {
// //         // No need for mutex if you know `read()` won't preempt
// //         printf("Angle: %.2f degrees\n", encoder.getAngle());
// //         vTaskDelay(10 / portTICK_PERIOD_MS); // Delay to print every second
// //     }
// // }

// // extern "C" void app_main() {

// //     motor_config.pole_pairs = 10;
// //     motor_config.max_voltage = 12.0;
// //     motor.on_configure(motor_config);

// //     inverter_config.enable_pin = 0;
// //     inverter_config.u_pin = 1;
// //     inverter_config.v_pin = 2;
// //     inverter_config.w_pin = 3;
// //     inverter_config.dc_voltage = 12.0;
// //     inverter.on_configure(inverter_config);

// //     controller.on_init(inverter, encoder, motor);

// //     encoder.init();
// //     xTaskCreate(&task_read_encoder, "task_read_encoder", 2048, NULL, 2, NULL);
// //     xTaskCreate(&print_encoder_angle, "print_encoder_angle", 2048, NULL, 2, NULL);
    

// // }



// #include <stdio.h>
// #include "esp_timer.h"
// #include "Controller.hpp"


// InverterConfig inverter_config;
// MotorConfig motor_config;
// Motor motor;
// Inverter inverter;
// FOCController controller;
// EncoderI2C encoder(6, 7, 0x36, 800000);

// void delay_ms(uint32_t ms) {
//     int64_t start_time = esp_timer_get_time(); // Get current time
//     while ((esp_timer_get_time() - start_time) < ms * 1000) {
//         // Busy wait until the delay time has passed
//     }
// }

// void delay_us(uint32_t us) {
//     int64_t start_time = esp_timer_get_time(); // Get current time in microseconds
//     while ((esp_timer_get_time() - start_time) < us) {
//         // Busy wait until the delay time has passed
//     }
// }

// extern "C" void app_main(void) {
//     // Configure the LEDC timer
//     inverter_config.u_pin = 2;
//     inverter_config.v_pin = 3;
//     inverter_config.w_pin = 4;
//     inverter_config.enable_pin = GPIO_NUM_5;
//     inverter_config.dc_voltage_ps = 12.0;
//     inverter_config.voltage_limit = 6.0;

//     motor_config.pole_pairs = 7;
//     motor_config.max_voltage = 3.0;
    
//     motor.on_configure(motor_config);
//     inverter.on_configure(inverter_config);
//     controller.on_init(inverter, encoder, motor);

//     delay_ms(5000);
//     printf("Ready\n");
//     printf("Starting\n");
//     delay_ms(1000);
//     controller.on_activate();
//     ESP_LOGI("Main", "Starting alignment");
//     float alignment_voltage = 3.0f;

//     // while(1){
//     //     controller.set_phase_voltage_(alignment_voltage, 0, 0);
//     // }
//         for (int i = 0; i <=500; i++ ) {
//       float angle = _3PI_2 + _2PI * i / 500.0f;
//       controller.set_phase_voltage_(alignment_voltage, 0, angle);
//       delay_ms(2);
//     }
// float ud = 0.0;  // D-axis voltage

//     ESP_LOGI("Main", "Alignment done");
//     controller.on_deactivate();    

// }


// FOC Controller using ESP-IDF LEDC
#include <cmath>
#include <iostream>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Controller.hpp"

FieldOrientedControl foc;
Inverter inverter;
FOCController controller;
Motor motor;
EncoderI2C encoder(6, 7, 0x36, 800000);
InverterConfig inverter_config;
MotorConfig motor_config;

constexpr float PI = 3.14159265358979f;
constexpr float V_SUPPLY = 12.0f;
constexpr int POLE_PAIRS = 7;
constexpr int GPIO_U = 2;
constexpr int GPIO_V = 3;
constexpr int GPIO_W = 4;
constexpr int GPIO_ENABLE = 5;



// FOC Controller
void focController(float uD, float uQ, float theta) {
    auto alfa_beta = foc.inversePark(uD, uQ, theta);
    auto voltages = foc.inverseClarke(alfa_beta);
    inverter.set_uvw(voltages.u, voltages.v, voltages.w);
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
    controller.on_init(inverter, encoder, motor);
    encoder.init();
    controller.align();

}



    // float uD = 0.0f; // Example input
    // float uQ = 1.0f; // Example input
    // float theta = PI / 4; // Example rotor angle
    // inverter.on_activate();
    // while (true) {
    //     focController(uD, uQ, theta);
    //     theta += 0.01f; // Simulating rotor rotation
    //     if (theta > 2 * PI) theta -= 2 * PI;
    //     vTaskDelay(pdMS_TO_TICKS(5)); // 10 ms delay
    // }