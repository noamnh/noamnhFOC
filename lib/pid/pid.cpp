// pid.cpp
#include "pid.hpp"
#include <algorithm>  // for std::clamp

PIDController::PIDController(float kp, float ki, float kd, float max_output, float min_output)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), min_output_(min_output),
      integral_(0.0f), prev_error_(0.0f) {}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}

float PIDController::compute(float desired, float measured, float dt) {
    float error = desired - measured;

    // Integrate with anti-windup
    integral_ += error * dt;

    // Derivative term
    float derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    // PID output
    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Clamp output
    output = std::clamp(output, min_output_, max_output_);

    return output;
}
