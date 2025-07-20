#include <cmath>
#pragma once

#define PI (3.141592f)
#define TWO_PI (6.283185f)
#define INV_TWO_PI (0.159155f)
#define EPSILON (2.718281f)
#define FAST_SINCOS_USE_INTERPOLATION 0

static const float one_by_sqrt3 = 0.57735026919f;
static const float two_by_sqrt3 = 1.15470053838f;
static const float threehalfpi = 4.7123889f;
static const float pi = PI;
static const float halfpi = PI * 0.5f;
static const float quarterpi = PI * 0.25f;

constexpr int TABLE_SIZE = 256;
inline float sin_table[TABLE_SIZE]; // Declare but do not define
inline float cos_table[TABLE_SIZE]; // Declare but do not define





static inline void generateSinCosTable() {
    for (int i = 0; i < TABLE_SIZE; i++) {
        float angle = (float)i * (2.0f * PI / TABLE_SIZE);
        sin_table[i] = sinf(angle);
        cos_table[i] = cosf(angle);
    }
}

static inline void fastSinCosInterpolated(float theta, float &sin_theta, float &cos_theta) {
    int index = (int)(theta * (TABLE_SIZE / (2.0f * PI))) % TABLE_SIZE;
    if (index < 0) index += TABLE_SIZE;

    float frac = theta * (TABLE_SIZE / (2.0f * PI)) - index;
    int next_index = (index + 1) % TABLE_SIZE;
    sin_theta = sin_table[index] * (1 - frac) + sin_table[next_index] * frac;
    cos_theta = cos_table[index] * (1 - frac) + cos_table[next_index] * frac;
}

static inline void fastSinCos(float theta, float &sin_theta, float &cos_theta) {
#if FAST_SINCOS_USE_INTERPOLATION
    fastSinCosInterpolated(theta, sin_theta, cos_theta);
#else
    int index = (int)(theta * (TABLE_SIZE / (2.0f * PI))) % TABLE_SIZE;
    if (index < 0) index += TABLE_SIZE;
    sin_theta = sin_table[index];
    cos_theta = cos_table[index];
#endif
}


inline float electrical_angle(float mechanical_angle, int pole_pairs) {
    // Convert mechanical angle to electrical angle
    return (float)(pole_pairs) * mechanical_angle;
}

inline float mechanical_angle(float electrical_angle, int pole_pairs) {
    // Convert electrical angle to mechanical angle
    return (float)(electrical_angle / pole_pairs);
}

inline float normalize_angle(float angle) {
    // Normalize angle to the range [0, 2π)
    angle = fmod(angle, TWO_PI);
    if (angle < 0) angle += TWO_PI; // Ensure positive angle
    return angle;
}

inline float calculate_electrical_angle(float mechanical_angle, int pole_pairs) {
    // Calculate the electrical angle from the mechanical angle
    return normalize_angle((float)(pole_pairs) * mechanical_angle);
}


inline float fast_sin(float theta) {
    float s, c;
    fastSinCosInterpolated(theta, s, c);
    return s;
}

inline float fast_cos(float theta) {
    float s, c;
    fastSinCosInterpolated(theta, s, c);
    return c;
}


static inline float clamp(float d, float min, float max)
{
    const float t = d < min ? min : d;
    return t > max ? max : t;
}

static inline int clampi(int d, int min, int max){
    const int t = d < min ? min : d;
    return t > max ? max : t;
}

static inline bool clampc(float *d, const float min, const float max)
{
    const float t = *d < min ? min : *d;
    *d = t > max ? max : t;
    return (*d == min) || (*d == max);
}

// Fast inverse square root (for modulation limiting)
static inline float fast_inv_sqrt(float x) {
    if (x <= 0.0f) return 1.0f;  // Avoid division by zero
    return 1.0f / sqrtf(x);
}


static inline int SVM(float alpha, float beta, float* tA, float* tB, float* tC)
{
    int Sextant;

    if (beta >= 0.0f)
    {
        if (alpha >= 0.0f)
        {
            //quadrant I
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        }
        else
        {
            //quadrant II
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    }
    else
    {
        if (alpha >= 0.0f)
        {
            //quadrant IV
            if (-one_by_sqrt3 * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        }
        else
        {
            //quadrant III
            if (one_by_sqrt3 * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant)
    {
        // sextant v1-v2
        case 1:
        {
            // Vector on-times
            float t1 = alpha - one_by_sqrt3 * beta;
            float t2 = two_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t1 - t2) * 0.5f;
            *tB = *tA + t1;
            *tC = *tB + t2;
        } break;

        // sextant v2-v3
        case 2:
        {
            // Vector on-times
            float t2 = alpha + one_by_sqrt3 * beta;
            float t3 = -alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t2 - t3) * 0.5f;
            *tA = *tB + t3;
            *tC = *tA + t2;
        } break;

        // sextant v3-v4
        case 3:
        {
            // Vector on-times
            float t3 = two_by_sqrt3 * beta;
            float t4 = -alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tB = (1.0f - t3 - t4) * 0.5f;
            *tC = *tB + t3;
            *tA = *tC + t4;
        } break;

        // sextant v4-v5
        case 4:
        {
            // Vector on-times
            float t4 = -alpha + one_by_sqrt3 * beta;
            float t5 = -two_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t4 - t5) * 0.5f;
            *tB = *tC + t5;
            *tA = *tB + t4;
        } break;

        // sextant v5-v6
        case 5:
        {
            // Vector on-times
            float t5 = -alpha - one_by_sqrt3 * beta;
            float t6 = alpha - one_by_sqrt3 * beta;

            // PWM timings
            *tC = (1.0f - t5 - t6) * 0.5f;
            *tA = *tC + t5;
            *tB = *tA + t6;
        } break;

        // sextant v6-v1
        case 6:
        {
            // Vector on-times
            float t6 = -two_by_sqrt3 * beta;
            float t1 = alpha + one_by_sqrt3 * beta;

            // PWM timings
            *tA = (1.0f - t6 - t1) * 0.5f;
            *tC = *tA + t1;
            *tB = *tC + t6;
        } break;
    }
    // if any of the results becomes NaN, result_valid will evaluate to false

    int result_valid =
            *tA >= 0.0f && *tA <= 1.0f
         && *tB >= 0.0f && *tB <= 1.0f
         && *tC >= 0.0f && *tC <= 1.0f;
    return result_valid ? 0 : -1;
}

static inline void inverse_park(float ud, float uq, float thetha, float &alpha, float &beta) {
    // Inverse Park transformation
        float sin_theta, cos_theta;
    fastSinCos(thetha, sin_theta, cos_theta);
    alpha = ud * cos_theta - uq * sin_theta;
    beta = ud * sin_theta + uq * cos_theta;
    // log the alpha and beta values
    // ESP_LOGI("inverse_park", "alpha: %f, beta: %f", alpha, beta);

}

static inline void inverse_clarke(float alpha, float beta, float &ia, float &ib, float &ic) {
    // Inverse Clarke transformation
    ia = alpha;
    ib = -0.5f * alpha + 0.86602540378f * beta;
    ic = -0.5f * alpha - 0.86602540378f * beta;
    // log the alpha and beta values
    // ESP_LOGI("inverse_clarke", "alpha: %f, beta: %f", alpha, beta);
}

static inline void clark_transform(float ia, float ib, float ic, float &i_alpha, float &i_beta) {
    i_alpha = ia;
    i_beta = one_by_sqrt3 * (ib - ic);
    // log the alpha and beta values
    // ESP_LOGI("clark_transform", "alpha: %f, beta: %f", alpha, beta);
}

static inline void park_transform(float i_alpha, float i_beta, float thetha, float &id, float &iq) {
    // Park transformation
    float sin_theta, cos_theta;
    fastSinCos(thetha, sin_theta, cos_theta);
    id = i_alpha * cos_theta + i_beta * sin_theta;
    iq = -i_alpha * sin_theta + i_beta * cos_theta;
    // log the id and iq values
    // ESP_LOGI("park_transform", "id: %f, iq: %f", id, iq);
    // ESP_LOGI("park_transform", "ud: %f, uq: %f", ud, uq);
}