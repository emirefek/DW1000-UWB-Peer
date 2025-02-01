#ifndef RANGING_H
#define RANGING_H

#include <cstdint>

// Speed of light constant in meters per second
const double SPEED_OF_LIGHT = 3.0e8;

// Function to calculate the distance based on UWB timestamps and processing time
double calculateDistance(uint64_t t1, uint64_t t4, uint64_t T_taxi);

#endif // RANGING_H
