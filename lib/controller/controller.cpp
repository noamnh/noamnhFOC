#include "controller.hpp"


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
    return ret;
}
esp_err_t Controller::on_deactivate() {
    // Deactivate the controller
    ESP_LOGI("Controller", "Deactivating controller...");
    return inv_.on_deactivate();
}



esp_err_t Controller::main_loop() {
    // Main loop for the controller

    esp_err_t ret = ESP_OK;

    if(enable_) { 
        xSemaphoreTake(update_semaphore_, portMAX_DELAY);
        
    
    }

    return ret;
}


bool Controller::update(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx) {
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(*((SemaphoreHandle_t*)user_ctx), &task_woken);
    return task_woken;
}


