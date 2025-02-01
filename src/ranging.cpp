#include "ranging.h"
#include <DW1000NgRanging.hpp>

double calculateDistance(uint64_t t1, uint64_t t4, uint64_t T_taxi)
{

    // Convert the raw difference directly to seconds.
    double tof_s = (static_cast<double>(t4) - static_cast<double>(t1) - static_cast<double>(T_taxi)) * 15.65e-12;
    // Distance (meters) = (tof_s * speed_of_light) / 2
    double distance = (tof_s * SPEED_OF_LIGHT) / 2.0;
    return distance;
}