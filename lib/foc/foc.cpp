#include "foc.hpp"

// TODO use fast sin and cos


FieldOrientedControl::FieldOrientedControl() {
    // Constructor
}

VoltagePhases FieldOrientedControl::inverseClarke(InverseParkOutput inverse_park) {
    VoltagePhases voltagePhases;
    voltagePhases.u = inverse_park.U_alfa;
    voltagePhases.v = 0.5f * (-inverse_park.U_alfa + _SQRT3 * inverse_park.U_beta);
    voltagePhases.w = 0.5f * (-inverse_park.U_alfa - _SQRT3 * inverse_park.U_beta);
    return voltagePhases;
}

void FieldOrientedControl::inversePark(float U_d, float U_q, float theta, float &U_alfa, float &U_beta) {
    float sin_theta, cos_theta;
    fastSinCos(theta, sin_theta, cos_theta);

    U_alfa = U_d * cos_theta - U_q * sin_theta;
    U_beta = U_d * sin_theta + U_q * cos_theta;

    // U_alfa = U_d * cos(theta) - U_q * sin(theta);
    // U_beta = U_d * sin(theta) + U_q * cos(theta);
}



ClarkeOutput FieldOrientedControl::clarke(float I_a, float I_b, float I_c) {
    ClarkeOutput clarke_output;
    clarke_output.I_alpha = I_a;
    clarke_output.I_beta = INV_SQRT3 * (I_a + 2.0f * I_b);
    return clarke_output;
}

ParkOutput FieldOrientedControl::park(ClarkeOutput clarke, float theta) {
    float sin_theta, cos_theta;
    fastSinCos(theta, sin_theta, cos_theta); // Use fast sin/cos lookup table

    ParkOutput park_output;
    park_output.I_d = clarke.I_alpha * cos_theta + clarke.I_beta * sin_theta;
    park_output.I_q = -clarke.I_alpha * sin_theta + clarke.I_beta * cos_theta;
    return park_output;
}

