#ifndef UTILS_H
#define UTILS_H

#include "config.h"

namespace utils {

// clamp value between min and max
float clamp(float value, float min_val, float max_val);

// quake III fast inverse square root
float fastInvSqrt(float number);

// qmi8658 sensor functions
bool initQMI8658();
void readQMI8658Accel(float* x, float* y, float* z);
void writeRegister(uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t reg);

}  // namespace utils

#endif