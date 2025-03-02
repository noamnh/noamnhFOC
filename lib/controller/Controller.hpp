#pragma once

#include "utils.hpp"
#include "foc.hpp"
#include "inverter.hpp"
#include "encoders.hpp"
#include "motor.hpp"
#include "cmath"
#include "esp_log.h"
#include "atomic"
#include "current_sense.hpp"
#include "LowPassFilter.hpp"

#define LOOP_FREQUENCY_HZ 10000 // 10 kHz
#define LOOP_PERIOD_TICKS (pdMS_TO_TICKS(1000) / LOOP_FREQUENCY_HZ)

// Controller gains
constexpr float Kp = 0.1f; // Proportional gain
constexpr float Ki = 0.01f; // Integral gain

// Saturation limits
constexpr float OUTPUT_MAX = 0.5f;
constexpr float OUTPUT_MIN = -0.5f;
constexpr float ALPHA = 0.1f; // Adjust based on desired filtering

enum class FOCState {
    STOPPED,
    RUNNING,
    ALIGNING
};

enum class ControlMode {
    Position = 0,
    Velocity = 1,
    Torque = 2,
    Trajectory = 3,
    Impedance = 4,
    Addmitance = 5
};

enum class SensorDirection {
    CW = 1,
    CCW = -1,
    NOT_DEFINED = 0
};


struct FOCControllerConfig{
    float dt = 1;
    float shaft_offset = 0;
    bool center_modulation = true;
    float alignment_voltage = 0.5;
};

class FOCController {
    public:

    FOCController();
        void set_phase_voltage_(float Uq, float Ud, float theta); // Uq, Ud, theta is the electrical angle

    void on_configure();
    void on_init(Inverter& inverter, EncoderI2C& encoder, Motor& motor, CurrentSense& current_sense);
    void on_activate();
    void on_deactivate();
    void run();
    void stop();
    bool align();

    float getShaftAngle();
    float getShaftVelocity();
    float getElectricalAngle();
    float getMechanicalAngle();
    
    void setSensorDirection(SensorDirection direction){sensor_direction_ = direction;}
    int getSensorDirection(){
        switch (sensor_direction_)
        {
        case SensorDirection::CW:
            return 1;
            break;
        case SensorDirection::CCW:
            return -1;
            break;

        case SensorDirection::NOT_DEFINED: // Use a default value
            // log warning message
            ESP_LOGW("FOCController", "Sensor direction not defined");
            return 1;
            break;
        }
        return 0;

    }

    


    SensorDirection findSensorDirection();

    bool findMotorParameters(const  MotorConfig &motor_config);
    
    bool findZeroElectricalAngle();

    float _normalizeAngle(float angle) {
        float a = fmod(angle, _2PI);
        return (a >= 0) ? a : (a + _2PI);
    }


    private:
    Inverter inverter_;
    EncoderI2C encoder_;
    Motor motor_;
    CurrentSense current_sense_;
    FieldOrientedControl foc_;
    LowPassFilter lpf_vel_;
    LowPassFilter lpf_iq_;
    LowPassFilter lpf_id_;
    LowPassFilter lpf_shaft_angle_;
    
    SensorDirection sensor_direction_ = SensorDirection::NOT_DEFINED;
    FOCState state_ = FOCState::STOPPED;
    FOCControllerConfig config_;
    bool center_modulation_ = true;
    std::atomic<float> sensor_offset_ = 0;
    float shaft_angle_ = 0;
    float shaft_velocity_ = 0;
    float mechanical_angle_ = 0;
    float zero_eletrical_angle_ = 0;
    float dt_ = 1;


    float normal_angle_(float angle);
    float electrical_angle_(float angle, int pole_pairs);
    float calculate_electrical_angle_(float shaft_angle);
    float calculate_shaft_velocity_(float shaft_angle);
    void velocity_control_(float desired_velocity);
    void test_after_align();
    void test_closed_loop_velocity();
    void current_closed_loop();
};
