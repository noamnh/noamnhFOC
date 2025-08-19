#include "controller.hpp"
#include <math.h>
#include "esp_timer.h"

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
    mcpwm_comparator_event_callbacks_t adc_mid_point_event_callbacks = {};
    adc_mid_point_event_callbacks.on_reach = Controller::update_adc_mid_point_event_callback;

        current_sense::Config config;
    config.shunt_resistor = 0.005f;
    config.amplifier_gain = 260.0f;  // Back to original calibrated value
    config.lpf_gain = 0.5f;


    current_observer_.on_init();
    current_observer_.on_configure(7, 8, 3, config);
    // Using the calculated amplifier gain of 200x
    current_observer_.on_activate();
    current_observer_.on_calibrate();

        // create the adc task


    inv_.on_init(&adc_mid_point_event_callbacks,this);
    mcpwm_timer_event_callbacks_t cbs = {};  // Initialize all fields to zero
    cbs.on_full = update; // Set the callback function for the timer event
    esp_err_t ret = inv_.set_inverter_callback(&cbs, this);


    sensor_.on_configure();
    sensor_.on_init();

    BaseType_t task_ret = xTaskCreatePinnedToCore(
    Controller::adc_task,         // Task function
    "adc_task",                   // Name
    2048,                         // Stack size
    this,                         // Task argument (this pointer)
    10,                           // Priority
    &adc_task_handle_,            // Task handle
    0                             // Core (ESP32-S3: core 0 recommended)
    );



    if (task_ret != pdPASS) {
        ESP_LOGE("Controller", "Failed to create ADC task: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }
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
    
    // Bus voltage normalization
    float one_over_Vbus_voltage = 1.0f / motor_.vbus_voltage;
    float mod_q = open_loop_uq_ * one_over_Vbus_voltage;
    float mod_d = open_loop_ud_ * one_over_Vbus_voltage;
    
    // Modulation limiting
    const float PWM_LIMIT = 0.95f;  // 95% modulation limit
    const float dq_mod_scale_factor = PWM_LIMIT * fast_inv_sqrt((mod_q * mod_q) + (mod_d * mod_d));
    
    if (dq_mod_scale_factor < 1.0f) {
        mod_q *= dq_mod_scale_factor;
        mod_d *= dq_mod_scale_factor;
    }
    
    inverse_park(mod_d, mod_q, angle, alpha, beta);
    SVM(alpha, beta, &ia, &ib, &ic);
    
    // Debug logging every 1000 steps
    static int step_counter = 0;
    // if (++step_counter % 1000 == 0) {
    //     ESP_LOGI("Controller", "PWM Duty: A=%.3f, B=%.3f, C=%.3f", ia, ib, ic);
    // }
    
    inv_.set_duty_cycle(ia, ib, ic);
}

void Controller::close_loop_step() {
    // get the current from the current observer
    // clark transform
    // park transform
    // lp filter
    // calculate the error
    // calculate the pid
    // calculate Vq and Vd 
    // normalize the Vq and Vd
    // I bus estimation 
    // inverse park transform
    // SVM
    // set the duty cycle   

}

esp_err_t Controller::main_loop() {
    esp_err_t ret = ESP_OK;
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second before starting the main loop

    // current_observer_.on_calibrate_ema();

    // calibrate_phase_resistance();
    // calibrate_phase_inductance();

    current_sum_log_.clear();
    current_sum_log_.reserve(1000); // Pre-allocate space for efficiency

    float ia, ib, ic;
    int64_t start_time = esp_timer_get_time(); // microseconds
    int64_t duration = 30 * 1000000; // 5 seconds in microseconds
    float current_sum = 0.0f;
    int counter = 0;  // <-- FIXED: normal variable, resets each call

    // example open loop step
    while(true){
        int64_t now = esp_timer_get_time();
        if ((now - start_time) >= duration) {
            break;
        }
                // xSemaphoreTake(update_semaphore_, portMAX_DELAY);
                //add delay 
                // vTaskDelay(pdMS_TO_TICKS(1)); // 1 ms delay for each step
                // timestamps_.pwm_period_us = esp_timer_get_time();
                // open_loop_step();
                // current_observer_.get_currents(ia, ib, ic);
                sensor_.update();

                // if (++counter % 1000 == 0) {

                //      current_sum = 0.5f * (fabsf(ia) + fabsf(ib) + fabsf(ic)) + 0.5f * current_sum;
                //     int64_t adc_us = timestamps_.adc_sample_time_us;
                //     int64_t pwm_us = timestamps_.pwm_period_us;
                //     // ESP_LOGI("Timing", "Δt (us) = %lld", adc_us - pwm_us);
                //     // ESP_LOGI("Controller", "Sensor angle: %.2f", sensor_.get_raw_angle());
                //     current_sum_log_.push_back(current_sum);
                // }

                // // log the raw angle
                float raw_angle = sensor_.get_raw_angle();
                ESP_LOGI("Controller", "Sensor raw angle: %.2f", raw_angle);
            
        }

        // Log all collected current sum values at the end
        ESP_LOGI("Controller", "=== Current Sum Log ===");
        ESP_LOGI("Controller", "Total samples: %zu", current_sum_log_.size());
        // ESP_LOGI("Controller", "Gain used: %f", config.amplifier_gain);

        float average_sum = 0.0f;

        for (size_t i = 0; i < current_sum_log_.size(); i++) {
            average_sum += current_sum_log_[i];
            // ESP_LOGI("Controller", "Sample %zu: Sum=%.3f A", i, current_sum_log_[i]);
        }

      average_sum /= current_sum_log_.size();
        ESP_LOGI("Controller", "Average Current Sum: %.3f A", average_sum);
        
        ESP_LOGI("Controller", "=== End Current Sum Log ===");
        current_sum_log_.clear();

        ESP_LOGI("Controller", "Open loop finished.");
        inv_.on_deactivate();

        return ret;


}


bool Controller::update(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx) {
    BaseType_t task_woken = pdFALSE;
    Controller* self = static_cast<Controller*>(user_ctx);  // ✅ FIXED
    self->timestamps_.pwm_period_us = esp_timer_get_time();       // PWM timer on_full
    xSemaphoreGiveFromISR(self->update_semaphore_, &task_woken);
    return task_woken;
}

bool IRAM_ATTR Controller::update_adc_mid_point_event_callback(mcpwm_cmpr_handle_t cmp, const mcpwm_compare_event_data_t *edata, void *user_ctx) {
    Controller* self = static_cast<Controller*>(user_ctx);  // ✅ FIXED
    BaseType_t task_woken = pdFALSE;
    self->timestamps_.adc_sample_time_us = esp_timer_get_time();  // ADC midpoint ISR
    BaseType_t ok = xTaskNotifyFromISR(self->adc_task_handle_, 0, eNoAction, &task_woken);
    // ESP_DRAM_LOGI("ISR", "Notified=%d, task_woken=%d", ok, task_woken);

    return task_woken == pdTRUE;
}

void Controller::adc_task(void* arg) {
    Controller* self = static_cast<Controller*>(arg);
    ESP_LOGI("Controller", "ADC task started, handle=%p", self->adc_task_handle_);
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait for notify from ISR
        // ESP_LOGI("Controller", "ADC task notified, processing data...");
        // Safe to process current here
        self->current_observer_.handle_dma_event();
        // self->sensor_.update();  // Update sensor data
    }
}

esp_err_t Controller::calibrate_phase_resistance() {

    // TODO make this configurable
    float ia = 0.0f, ib = 0.0f, ic = 0.0f;
    float actual_current = 0.0f;
    float v_bus = 12.0f;
    float v_target = 1.0f;
    float max_voltage = 5.0f;
    float min_current = 0.5f;
    float target_current = 0.8f;

    // inv_.on_activate(); // Turn on inverter (set up PWM timers, etc.)

    for (int i = 0; i < 2 * PWM_FREQUENCY; i++) {
        xSemaphoreTake(update_semaphore_, portMAX_DELAY); // Wait for PWM update

        // Slowly ramp up target voltage
        v_target += 0.01f;
        if (v_target > max_voltage) {
            ESP_LOGI("Controller", "Max voltage reached: %.2f V", max_voltage);
            break;
        }

        float duty = v_target / v_bus;
        inv_.set_duty_cycle(duty, -duty, 0.0f); // AB active, C floating

        current_observer_.get_currents(ia, ib, ic);


        // Current in AB line is approximately ia or -ib (assuming symmetry)
        actual_current = fabsf(ia); // or (fabsf(ia) + fabsf(ib)) * 0.5f

        // Stop when you reach target current
        if (actual_current >= target_current && actual_current >= min_current) {
            ESP_LOGI("Controller", "Target current reached: %.3f A", actual_current);
            break;
        }
    }

    // Shut down PWM and inverter
    inv_.set_duty_cycle(0.0f, 0.0f, 0.0f);
    // inv_.on_deactivate();

    // Final resistance calculation (single phase)
    float R = v_target / (2.0f * actual_current);
    ESP_LOGI("Controller", "Phase resistance: %.4f Ω", R);

    // Store or return R as needed
    return ESP_OK;
}

esp_err_t Controller::calibrate_phase_inductance() {


    // TODO make this configurable

float v_bus = 12.0f;
float v_applied = 1.0f;
float duty = v_applied / v_bus;
int num_samples = 8;
float L_sum = 0.0f;
int dt_us = 50;
int actual_sample = 0;
float ib = 0.0f, ic = 0.0f;
uint64_t pulse_start_time = 0;
bool pulse_active = false;

inv_.set_duty_cycle(0.0f, 0.0f, 0.0f); // Ensure motor is idle

float ia_start = 0.0f, ia_end = 0.0f;

while (actual_sample < num_samples) {
    // Wait for control loop sync (1 PWM cycle)
    xSemaphoreTake(update_semaphore_, portMAX_DELAY);

    if (!pulse_active) {
        // Step 1: Get current before pulse (already sampled in ISR)
        current_observer_.get_currents(ia_start, ib, ic);

        // Step 2: Start pulse
        inv_.set_duty_cycle(duty, -duty, 0.0f);

        // Step 3: Start timer
        pulse_start_time = esp_timer_get_time();
        pulse_active = true;
    } else {
        // Step 4: Wait until pulse duration elapsed
        if ((esp_timer_get_time() - pulse_start_time) >= dt_us) {
            // Step 5: Get current after pulse (already updated in last ISR)
            current_observer_.get_currents(ia_end, ib, ic);

            // Step 6: Stop pulse
            inv_.set_duty_cycle(0.0f, 0.0f, 0.0f);

            // Step 7: Compute inductance
            float di = fabsf(ia_end - ia_start);
            if (di > 0.005f) { // Avoid division by noise
                float L = (v_applied * dt_us * 1e-6f) / di;
                L_sum += L;
                actual_sample++;
            }

            pulse_active = false; // Ready for next sample
        }
    }
}
   

    

    inv_.set_duty_cycle(0.0f, 0.0f, 0.0f);

    float L_avg = L_sum / actual_sample;
    ESP_LOGI("Controller", "Phase inductance: %.2f uH (avg of %d)", L_avg * 1e6, num_samples);
    return ESP_OK;
}