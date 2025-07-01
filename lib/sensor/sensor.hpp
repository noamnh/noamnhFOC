
#include "esp_log.h"
#include "utils.h"


typedef struct {
    uint16_t ticks;
    float raw_angle;
    float prev_angle; // Previous angle for velocity calculation
    float multiturn_angle;
    float velocity;
    uint32_t timestamp; // Timestamp in microseconds
} sensor_state_t;

typedef struct {
    bool is_calibrated;
    uint32_t error;
} sensor_status_t;

typedef struct {
    float offset; // Offset for angle correction
    uint16_t resolution; // Resolution of the sensor in ticks
} sensor_config_t;


class Sensor {
public:
    Sensor() = default;

    virtual void on_init() = 0;
    virtual void on_configure() = 0;
    virtual void update() = 0;
    virtual void reset() = 0;
    virtual bool calibrate() = 0;
    virtual float get_raw_angle() const = 0; // get the raw single turn angle
    virtual float get_ticks() const = 0; // get the raw ticks
    virtual float get_multiturn_angle() const = 0; // get the multiturn angle
    virtual float get_velocity() const = 0; // get the velocity

    
    protected:
        sensor_state_t state_;
    sensor_status_t status_;
    sensor_config_t config_;

    private:


};