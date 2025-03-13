#pragma once

#include "esp_timer.h"
#include <cmath>

#define _SQRT3_2 0.86602540378f
#define _PI_2 1.57079632679f
#define _2PI 6.28318530718f
#define _PI 3.14159265359f
#define _3PI_2 4.71238898038f
#define _SQRT3 1.73205080757f
#define DEG_TO_RAD 0.01745329251
#define RAD_TO_DEG 57.2957795131
#define INV_SQRT3 0.57735026919f // Precomputed 1/sqrt(3)

template <typename T>
T clamp(T value, T min_val, T max_val) {
    return (value < min_val) ? min_val : (value > max_val) ? max_val : value;
}

inline float sqrt_(float number) {
    union {
        float    f;
        uint32_t i;
    } y = { .f = number };
    y.i = 0x5f375a86 - ( y.i >> 1 );
    return number * y.f;
}

// Function to get current time in milliseconds
inline uint64_t get_millis() {
    return esp_timer_get_time() / 1000; // Convert microseconds to milliseconds
}

// Function to get current time in microseconds
inline uint64_t get_micros() {
    return esp_timer_get_time(); // Already in microseconds
}

constexpr int TABLE_SIZE = 256;
extern float sin_table[TABLE_SIZE]; // Declare but do not define
extern float cos_table[TABLE_SIZE]; // Declare but do not define

void generateSinCosTable();
void fastSinCos(float theta, float &sin_theta, float &cos_theta);
