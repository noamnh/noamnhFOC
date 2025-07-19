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


inverter::Inverter inv;
Controller controller;

bool update_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx){
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(*((SemaphoreHandle_t*)user_ctx), &task_woken);
    return task_woken;
}


extern "C" void app_main() {


controller.on_init();
controller.on_activate();
float ud = 0.0;
float uq = 0.05;
float speed_dps = 720; // desired speed in degrees per second
float speed_rps = speed_dps * PI / 180.0f; // rad/sec
float sample_time_sec = 1.0f / 1000.0f; // your loop period (2 ms)
controller.set_open_loop_params(speed_dps, uq, ud, sample_time_sec);
controller.main_loop();

}

