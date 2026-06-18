#include "pca9685.h"
#include "servo_calibration.h"

#include <assert.h>

static void test_pulse_conversion()
{
    assert(pca9685_pulse_microseconds_to_ticks(1000) == 204);
    assert(pca9685_pulse_microseconds_to_ticks(1500) == 307);
    assert(pca9685_pulse_microseconds_to_ticks(2000) == 409);

    assert(pca9685_ticks_to_pulse_microseconds(204) == 996);
    assert(pca9685_ticks_to_pulse_microseconds(307) == 1499);
    assert(pca9685_ticks_to_pulse_microseconds(409) == 1997);
}

static void test_servo_limits()
{
    ServoPulseLimits limits;
    assert(servo_get_pulse_limits(0, &limits));
    assert(limits.min_microsec == 380);
    assert(limits.max_microsec == 1700);

    assert(servo_get_pulse_limits(11, &limits));
    assert(limits.min_microsec == 1000);
    assert(limits.max_microsec == 2100);

    assert(!servo_is_valid_index(-1));
    assert(!servo_is_valid_index(SERVO_COUNT));
    assert(!servo_get_pulse_limits(SERVO_COUNT, &limits));
}

static void test_servo_clamping()
{
    assert(servo_clamp_pulse_microseconds(0, 100) == 380);
    assert(servo_clamp_pulse_microseconds(0, 1200) == 1200);
    assert(servo_clamp_pulse_microseconds(0, 3000) == 1700);

    assert(servo_clamp_pulse_microseconds(11, 500) == 1000);
    assert(servo_clamp_pulse_microseconds(11, 1500) == 1500);
    assert(servo_clamp_pulse_microseconds(11, 3000) == 2100);
}

int main()
{
    test_pulse_conversion();
    test_servo_limits();
    test_servo_clamping();
    return 0;
}
