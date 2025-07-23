#include "esp_inverter.hpp"
#include "current_observer.hpp"

#include "utils.h"
#include "freertos/FreeRTOS.h"
#include "as5600.hpp"
#include <vector>
using namespace inverter;



enum Mode {
    ERROR = -1,
    OPEN_LOOP = 0,
    CALIBRATE = 1,
    IDLE = 2,
    CURRENT_CONTROL = 3,
    VELOCITY_CONTROL = 4,
    POSITION_CONTROL = 5,
    STOP = 6,

};

struct Gains{
    float Kp = 0.0f; // Proportional gain
    float Ki = 0.0f; // Integral gain
    float Kd = 0.0f; // Derivative gain
    float Kff = 0.0f; // Feedforward gain
};

struct ControllerConfig {
    float lpf_gain = 0.3f;
    float plant_pole = 0.0f;
    float bandwidth = 20000.0f; // 20kHz
    float current_limit = 4.0f; // 4A
    float velocity_limit = 1000.0f; // 1000 degrees per second

};

struct MotorConfig {
    int pole_pairs;
    float phase_resistance;
    float phase_inductance;
    float vbus_voltage = 12.0f;
};

struct Setpoint {
    float position_dps = 0.0f;
    float velocity_dps = 0.0f;
    float vq = 0.0f;
    float vd = 0.0f;
    float id = 0.0f;
    float iq = 0.0f;
};

struct Timestamps {
    int64_t pwm_period_us = 0; // PWM period in microseconds
    int64_t adc_sample_time_us = 0; // ADC sample time in microseconds
    int64_t control_loop_time_us = 0; // Control loop time in microseconds

};

class Controller {
public:
    Controller();
    ~Controller();

    esp_err_t on_configure();
    esp_err_t on_init();
    esp_err_t on_activate();
    esp_err_t on_deactivate();
    static bool update(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx);

    // bool IRAM_ATTR adc_mid_point_event_callback(mcpwm_cmpr_handle_t cmp, const mcpwm_compare_event_data_t *edata);
    static bool IRAM_ATTR update_adc_mid_point_event_callback(mcpwm_cmpr_handle_t cmp, const mcpwm_compare_event_data_t *edata, void *user_ctx);

    esp_err_t find_sensor_direction();
    esp_err_t find_sensor_offset();
    esp_err_t open_loop(float speed_dps);
    esp_err_t main_loop();

    // Set open-loop parameters
    void set_open_loop_params(float speed_dps, float uq, float ud = 0.0f, float sample_time_sec = 1.0f/1000.0f);

    // Set controller mode
    void set_mode(Mode mode);

    esp_err_t calibrate_phase_resistance();
    esp_err_t calibrate_phase_inductance();
    esp_err_t calibrate_sensor_offset();
    esp_err_t calibrate_current_offsets();
    esp_err_t calibrate_pole_pairs();

private:
    Inverter inv_;
    AS5600 sensor_;
    current_sense::CurrentObserver current_observer_;

    bool enable_ = false; // Flag to enable or disable the controller
    SemaphoreHandle_t update_semaphore_ = nullptr; // Semaphore for synchronization
    MotorConfig motor_;
    Setpoint setpoint_;
    Mode mode_ = STOP; // Current mode of the controller

    // Open-loop parameters
    float open_loop_speed_dps_ = 720.0f;
    float open_loop_uq_ = 0.05f;
    float open_loop_ud_ = 0.0f;
    float open_loop_sample_time_sec_ = 1.0f / 1000.0f;
    float open_loop_rad_per_sample_ = 720.0f * 3.14159265358979323846f / 180.0f * (1.0f / 1000.0f); // default

    // Vector to store current sum values for logging
    std::vector<float> current_sum_log_;

    void open_loop_step();
    void close_loop_step();

    TaskHandle_t adc_task_handle_ = nullptr; // task handle for ADC sampling syncrionized and trigger by midpoint of pwm
    static void adc_task(void* arg);

    Timestamps timestamps_; // Timestamps for various operations


};


