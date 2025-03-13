#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float max_output, float min_output);

    float compute(float desired, float measured);

private:
    float kp_, ki_, kd_;
    float max_output_, min_output_;
    float integral_, prev_error_;
};
