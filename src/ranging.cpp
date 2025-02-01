#include "ranging.h"

double rawToMicroseconds(uint64_t raw)
{
    // Each raw unit is roughly 15.65 ps. // Multiply by 15.65e-12 seconds and convert to microseconds:
    return static_cast<double>(raw) * 15.65e-12 * 1e6;
}

double calculateDistance(uint64_t t1, uint64_t t4, uint64_t T_taxi)
{
    double t1_us = rawToMicroseconds(t1);
    double t4_us = rawToMicroseconds(t4);
    double T_taxi_us = rawToMicroseconds(T_taxi);
    double tof_s = (t4_us - t1_us - T_taxi_us) * 1e-6; // convert microseconds to seconds

    // Distance = time-of-flight * speed of light / 2
    double distance = (tof_s * 299702547.0) / 2.0;

    return distance;
}