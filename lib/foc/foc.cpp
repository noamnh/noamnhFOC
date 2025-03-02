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

InverseParkOutput FieldOrientedControl::inversePark(float U_d, float U_q, float theta) {
    InverseParkOutput inverse_park;
    inverse_park.U_alfa = U_d * cos(theta) - U_q * sin(theta);
    inverse_park.U_beta = U_d * sin(theta) + U_q * cos(theta);
    return inverse_park;
}


ClarkeOutput FieldOrientedControl::clarke(CurrentPhases currentPhases) {
    ClarkeOutput clarke_output;
    clarke_output.I_alpha = currentPhases.I_a;
    clarke_output.I_beta = 1 / sqrt(3) * currentPhases.I_a + 2 / sqrt(3) * currentPhases.I_b;
    return clarke_output;
}

ParkOutput FieldOrientedControl::park(ClarkeOutput clarke, float theta) {
    ParkOutput park_output;
    park_output.I_d = clarke.I_alpha * cos(theta) + clarke.I_beta * sin(theta);
    park_output.I_q = -clarke.I_alpha * sin(theta) + clarke.I_beta * cos(theta);
    return park_output;
}

