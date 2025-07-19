#include "controller.hpp"
#include <math.h>

Controller::Controller() {
    // Constructor implementation
    ESP_LOGI("Controller", "Controller instance created.");
}
Controller::~Controller() {
    // Destructor implementation
    ESP_LOGI("Controller", "Controller instance destroyed.");
}

esp_err_t Controller::on_configure() {
    // Configure the controller
    ESP_LOGI("Controller", "Configuring controller...");
    return ESP_OK;
}

esp_err_t Controller::on_init() {
    // Initialize the controller
    ESP_LOGI("Controller", "Initializing controller...");
    generateSinCosTable(); // Generate sine and cosine tables for FOC
    update_semaphore_ = xSemaphoreCreateCounting(1, 0);
    inv_.on_init();
    mcpwm_timer_event_callbacks_t cbs = {};  // Initialize all fields to zero
    cbs.on_full = update; // Set the callback function for the timer event
    esp_err_t ret = inv_.set_inverter_callback(&cbs, &update_semaphore_);
    return ESP_OK;
}
esp_err_t Controller::on_activate() {
    ESP_LOGI("Controller", "Activating controller...");
    esp_err_t ret = inv_.on_activate();
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second before starting the main loop

    enable_ = true;
    return ret;
}
esp_err_t Controller::on_deactivate() {
    // Deactivate the controller
    ESP_LOGI("Controller", "Deactivating controller...");
    enable_ = false;
    return inv_.on_deactivate();
}

void Controller::set_open_loop_params(float speed_dps, float uq, float ud, float sample_time_sec) {
    open_loop_speed_dps_ = speed_dps;
    open_loop_uq_ = uq;
    open_loop_ud_ = ud;
    open_loop_sample_time_sec_ = sample_time_sec;
    float speed_rps = open_loop_speed_dps_ * PI / 180.0f;
    open_loop_rad_per_sample_ = speed_rps * open_loop_sample_time_sec_;
}

void Controller::set_mode(Mode mode) {
    mode_ = mode;
}

// --- Open Loop Step ---
void Controller::open_loop_step() {
    static float angle = 0.0f;
    float alpha, beta, ia, ib, ic;
    angle += open_loop_rad_per_sample_;
    if (angle > 2.0f * M_PI) {
        angle -= 2.0f * M_PI;
    }
    inverse_park(open_loop_ud_, open_loop_uq_, angle, alpha, beta);
    SVM(alpha, beta, &ia, &ib, &ic);
    inv_.set_duty_cycle(ia, ib, ic);
}

esp_err_t Controller::main_loop() {
    esp_err_t ret = ESP_OK;
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second before starting the main loop
    // float ud = 0.0;
    // float uq = 0.05;
    // float speed_dps = 720; // desired speed in degrees per second
    // float speed_rps = speed_dps * PI / 180.0f; // rad/sec
    // float sample_time_sec = 1.0f / 1000.0f; // your loop period (2 ms)
    // set_open_loop_params(speed_dps, uq, ud, sample_time_sec);
        while(true){
                xSemaphoreTake(update_semaphore_, portMAX_DELAY);
                open_loop_step();
        }
        return ret;


}


bool Controller::update(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx) {
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(*((SemaphoreHandle_t*)user_ctx), &task_woken);
    return task_woken;
}


