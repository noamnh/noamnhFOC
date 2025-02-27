#include "LowPassFilter.hpp"


LowPassFilter::LowPassFilter(){
    // Constructor
}

LowPassFilter::LowPassFilter(float alpha){
    // Constructor
    alpha_ = alpha;
}

void LowPassFilter::on_init(float value){
    // Initialize the filter
    prev_value_ = value;
}

float LowPassFilter::filter(float value){
    // Filter the value
    float filtered_value = alpha_ * value + (1 - alpha_) * prev_value_;
    prev_value_ = filtered_value;
    return filtered_value;
}

void LowPassFilter::setAlpha(float alpha){
    // Set the alpha value
    alpha_ = alpha;
}

