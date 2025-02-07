#include "motor.hpp"

Motor::Motor() {
    // Constructor
}

void Motor::on_configure(MotorConfig motor_config) {
    // Configure the motor
    config_ = motor_config;
}

MotorConfig Motor::get_config() {
    // Get the motor configuration
    return config_;
}