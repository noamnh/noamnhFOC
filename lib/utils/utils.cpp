#include "utils.hpp"
#include <cmath>

float sin_table[TABLE_SIZE]; // Define the global array
float cos_table[TABLE_SIZE]; // Define the global array

void generateSinCosTable() {
    for (int i = 0; i < TABLE_SIZE; i++) {
        float angle = (float)i * (2.0f * _PI / TABLE_SIZE);
        sin_table[i] = sinf(angle);
        cos_table[i] = cosf(angle);
    }
}

void fastSinCos(float theta, float &sin_theta, float &cos_theta) {
    int index = (int)(theta * (TABLE_SIZE / (2.0f * _PI))) % TABLE_SIZE;
    if (index < 0) index += TABLE_SIZE;
    sin_theta = sin_table[index];
    cos_theta = cos_table[index];
}
