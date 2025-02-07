
#include "utils.hpp"

struct InverseParkOutput{
    float U_alfa = 0;
    float U_beta = 0;
};

struct ParkOutput{
    float I_q = 0;
    float I_d = 0;
};

struct VoltagePhases {
    float u = 0;
    float v = 0;
    float w = 0;
};

struct CurrentPhases {
    float I_a = 0;
    float I_b = 0;
    float I_c = 0;
};

struct ClarkeOutput {
    float I_alpha = 0;
    float I_beta = 0;
};




class FieldOrientedControl {

public:

    FieldOrientedControl();

    VoltagePhases inverseClarke(InverseParkOutput inverse_park);
    InverseParkOutput inversePark(float U_d, float U_q, float theta);
    ClarkeOutput clarke(CurrentPhases currentPhases);
    ParkOutput park(VoltagePhases voltagePhases, float theta);


private:


};