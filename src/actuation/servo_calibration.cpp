#include "servo_calibration.h"

static const ServoPulseLimits SERVO_LIMITS[SERVO_COUNT] = {
    { 380, 2800 },
    // 0 { 380, 1700 },
    { 380, 2800 },
    // 1 { 380, 1500 },
    { 380, 1800 },
    // 2 { 380, 1500 },
    { 380, 2100 },
    // 3 { 380, 1700 },
    { 380, 1800 },
    // 4 { 380, 1400 },
    { 380, 1800 },
    // 5 { 500, 1500 },
    { 380, 2800 },
    // 6 { 500, 1400 },
    { 380, 2500 },
    // 7 { 380, 1500 },
    { 380, 1800 },
    // 8 { 550, 1650 },
    { 380, 2300 },
    // 9 { 380, 1900 },
    { 380, 1800 },
    // 10 { 650, 1500 },
    { 380, 2000 },
    // 11 { 1000, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
    // { 380, 2100 },
};

bool servo_is_valid_index(int servo_index)
{
    return servo_index >= 0 && servo_index < SERVO_COUNT;
}

bool servo_get_pulse_limits(int servo_index, ServoPulseLimits *limits)
{
    if (!servo_is_valid_index(servo_index) || limits == 0)
    {
        return false;
    }

    *limits = SERVO_LIMITS[servo_index];
    return true;
}

int servo_clamp_pulse_microseconds(int servo_index, int pulse_microsec)
{
    ServoPulseLimits limits;
    if (!servo_get_pulse_limits(servo_index, &limits))
    {
        return pulse_microsec;
    }

    if (pulse_microsec < limits.min_microsec)
    {
        return limits.min_microsec;
    }
    if (pulse_microsec > limits.max_microsec)
    {
        return limits.max_microsec;
    }

    return pulse_microsec;
}
