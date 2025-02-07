#pragma once


class LowPassFilter{

public:
    LowPassFilter();
    LowPassFilter(float alpha);
    float filter(float value);
    void setAlpha(float alpha);
    float getAlpha();

    private: 
    float alpha_; // alpha value of the filter
    float prev_value_ = 0; // previous value of the filter
};