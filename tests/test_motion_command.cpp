#include "motion_command.h"

#include <assert.h>
#include <math.h>

static void test_zero_command()
{
    bool valid = false;
    MotionCommand command = motion_command_zero();
    MotionCommand bounded = motion_command_sanitize(&command, 0, &valid);

    assert(valid);
    assert(bounded.velocity_x_mps == 0.0f);
    assert(bounded.velocity_y_mps == 0.0f);
    assert(bounded.yaw_rate_radps == 0.0f);
}

static void test_clamping()
{
    bool valid = false;
    MotionCommand command;
    command.velocity_x_mps = 99.0f;
    command.velocity_y_mps = -99.0f;
    command.yaw_rate_radps = 99.0f;

    MotionCommandLimits limits = motion_command_default_limits();
    MotionCommand bounded = motion_command_sanitize(&command, &limits, &valid);

    assert(valid);
    assert(bounded.velocity_x_mps == limits.max_velocity_x_mps);
    assert(bounded.velocity_y_mps == -limits.max_velocity_y_mps);
    assert(bounded.yaw_rate_radps == limits.max_yaw_rate_radps);
}

static void test_deadzone()
{
    bool valid = false;
    MotionCommand command;
    command.velocity_x_mps = 0.0005f;
    command.velocity_y_mps = -0.0005f;
    command.yaw_rate_radps = 0.0005f;

    MotionCommand bounded = motion_command_sanitize(&command, 0, &valid);

    assert(valid);
    assert(bounded.velocity_x_mps == 0.0f);
    assert(bounded.velocity_y_mps == 0.0f);
    assert(bounded.yaw_rate_radps == 0.0f);
}

static void test_non_finite()
{
    bool valid = true;
    MotionCommand command;
    command.velocity_x_mps = NAN;
    command.velocity_y_mps = 0.0f;
    command.yaw_rate_radps = 0.0f;

    MotionCommand bounded = motion_command_sanitize(&command, 0, &valid);

    assert(!valid);
    assert(bounded.velocity_x_mps == 0.0f);
    assert(bounded.velocity_y_mps == 0.0f);
    assert(bounded.yaw_rate_radps == 0.0f);
}

int main()
{
    test_zero_command();
    test_clamping();
    test_deadzone();
    test_non_finite();
    return 0;
}
