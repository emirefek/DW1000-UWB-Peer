#ifndef RANGING_H
#define RANGING_H

#include <cstdint>

// Speed of light in meters per second, chosen to match DW1000 settings.
const double SPEED_OF_LIGHT = 299702547.0;

// Converts raw DW1000 timestamp units to microseconds.
// Each raw unit is ~15.65 ps, so:
//   microseconds = raw * 15.65e-12 sec/tick * 1e6 µs/sec = raw * 15.65e-6 µs/tick
double rawToMicroseconds(uint64_t raw);

// Calculates the distance in meters based on DW1000 timestamps.
// t1: transmit timestamp of the BLINK message (raw DW1000 units)
// t4: receive timestamp of the ANSWER message (raw DW1000 units)
// T_taxi: processing delay in raw DW1000 units (t3 - t2)
double calculateDistance(uint64_t t1, uint64_t t4, uint64_t T_taxi);

#endif // RANGING_H