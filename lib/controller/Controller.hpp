#pragma once

#include "utils.hpp"
#include "foc.hpp"
#include "inverter.hpp"
#include "encoders.hpp"
#include "motor.hpp"
#include "cmath"
#include "esp_log.h"



#define LOOP_FREQUENCY_HZ 10000 // 10 kHz
#define LOOP_PERIOD_TICKS (pdMS_TO_TICKS(1000) / LOOP_FREQUENCY_HZ)


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
    float alignment_voltage = 2;
};

class FOCController {
    public:

    FOCController();
        void set_phase_voltage_(float Uq, float Ud, float theta); // Uq, Ud, theta is the electrical angle

    void on_configure();
    void on_init(Inverter& inverter, EncoderI2C& encoder, Motor& motor);
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



    private:
    Inverter inverter_;
    EncoderI2C encoder_;
    Motor motor_;
    FieldOrientedControl foc_;
    SensorDirection sensor_direction_ = SensorDirection::NOT_DEFINED;
    FOCState state_ = FOCState::STOPPED;
    FOCControllerConfig config_;
    bool center_modulation_ = true;
    float sensor_offset_ = 0;
    float shaft_angle_ = 0;
    float shaft_velocity_ = 0;
    float electrical_angle_ = 0;
    float mechanical_angle_ = 0;
    float zero_eletrical_angle_ = 0;
    float dt_ = 1;


    float normal_angle_(float angle);
    float calculate_electrical_angle_(float shaft_angle);
    float calculate_shaft_angle_(float angle);
    float calculate_shaft_velocity_(float shaft_angle);

};
