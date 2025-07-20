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
// current_sense::CurrentObserver current_observer;

bool update_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx){
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(*((SemaphoreHandle_t*)user_ctx), &task_woken);
    return task_woken;
}


extern "C" void app_main() {

// current_observer.on_init();
// current_observer.on_configure(7, 8, 3);
// current_observer.set_shunt_and_gain(0.005f, 1.0f);
// current_observer.on_activate();
// current_observer.on_calibrate();

controller.on_init();
controller.on_activate();
float ud = 0.0;
float uq = 1.0f;  // 2V out of 12V = 16.7% duty cycle
float speed_dps = 720; // desired speed in degrees per second
float speed_rps = speed_dps * PI / 180.0f; // rad/sec
float sample_time_sec = 1.0f / 20000.0f; // PWM frequency = 20kHz
controller.set_open_loop_params(speed_dps, uq, ud, sample_time_sec);
controller.main_loop();



}

