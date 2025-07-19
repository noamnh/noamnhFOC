#include "esp_inverter.hpp"
#include "utils.h"
#include "freertos/FreeRTOS.h"
#include "as5600.hpp"
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

struct motor {
    int pole_pairs;
    float resistance;
    float inductance;
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

private:
    Inverter inv_;
    AS5600 sensor_;

    bool enable_ = false; // Flag to enable or disable the controller
    SemaphoreHandle_t update_semaphore_ = nullptr; // Semaphore for synchronization
    motor motor_;
    Mode mode_ = STOP; // Current mode of the controller

    // Open-loop parameters
    float open_loop_speed_dps_ = 720.0f;
    float open_loop_uq_ = 0.05f;
    float open_loop_ud_ = 0.0f;
    float open_loop_sample_time_sec_ = 1.0f / 1000.0f;
    float open_loop_rad_per_sample_ = 720.0f * 3.14159265358979323846f / 180.0f * (1.0f / 1000.0f); // default

    void open_loop_step();
};


