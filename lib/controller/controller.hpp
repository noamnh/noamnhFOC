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

struct config{
    float lpf_gain = 0.3f;
};

struct MotorConfig {
    int pole_pairs;
    float resistance;
    float inductance;
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

class Controller {
public:
    Controller();
    ~Controller();

    esp_err_t on_configure();
    esp_err_t on_init();
    esp_err_t on_activate();
    esp_err_t on_deactivate();
    static bool update(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx);

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
};


