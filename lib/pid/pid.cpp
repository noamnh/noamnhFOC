#include "pid.hpp"

PIDController::PIDController(float kp, float ki, float kd, float max_output, float min_output)
    : kp_(kp), ki_(ki), kd_(kd), max_output_(max_output), min_output_(min_output),
      integral_(0), prev_error_(0) {}

float PIDController::compute(float desired, float measured) {
    float error = desired - measured;
    integral_ += error;
    float derivative = error - prev_error_;
    prev_error_ = error;

    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // Clamp output
    if (output > max_output_) output = max_output_;
    if (output < min_output_) output = min_output_;

    return output;
}
