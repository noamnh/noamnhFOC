#include <esp_log.h>
// #include "as5600.hpp"

// Forward declarations
#include <math.h>
// #include "foc.hpp"
// #include "freertos/FreeRTOS.h"
// #include "freertos/semphr.h"
// #include "driver/gpio.h"
// #include "inverter.hpp"
// #include "esp_inverter.hpp"

#include "controller.hpp"
// #include "current_observer.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


inverter::Inverter inv;
Controller controller;




void foc_main_task(void* arg) {
    controller.main_loop();
    vTaskDelete(nullptr);
}

extern "C" void app_main() {



controller.on_init();
controller.on_activate();
float ud = 0.0;
float uq = 1.0f;  // 2V out of 12V = 16.7% duty cycle
float speed_dps = -5000; // desired speed in degrees per second
float speed_rps = speed_dps * PI / 180.0f; // rad/sec
float sample_time_sec = 1.0f / 20000.0f; // PWM frequency = 20kHz
controller.set_open_loop_params(speed_dps, uq, ud, sample_time_sec);
// controller.main_loop();

    xTaskCreatePinnedToCore(
        foc_main_task, "foc_main_task", 4096, nullptr, 5, nullptr, 1);


}

