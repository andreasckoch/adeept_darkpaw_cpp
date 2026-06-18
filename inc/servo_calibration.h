#ifndef SERVO_CALIBRATION_H_
#define SERVO_CALIBRATION_H_

#include <stdbool.h>

static const int SERVO_COUNT = 12;

struct ServoPulseLimits
{
    int min_microsec;
    int max_microsec;
};

bool servo_is_valid_index(int servo_index);
bool servo_get_pulse_limits(int servo_index, ServoPulseLimits *limits);
int servo_clamp_pulse_microseconds(int servo_index, int pulse_microsec);

#endif /* SERVO_CALIBRATION_H_ */
