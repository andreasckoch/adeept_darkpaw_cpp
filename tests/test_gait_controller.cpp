#include "gait_controller.h"

#include <assert.h>

static void assert_targets_valid(const GaitDryRunResult *result)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        ServoPulseLimits limits;
        assert(servo_get_pulse_limits(i, &limits));
        assert(result->targets[i].channel == i);
        assert(result->targets[i].pulse_microsec >= limits.min_microsec);
        assert(result->targets[i].pulse_microsec <= limits.max_microsec);
        assert(result->targets[i].ticks >= 0);
        assert(result->targets[i].within_limits);
    }
}

static void test_zero_command_stands()
{
    MotionCommand command = motion_command_zero();
    GaitDryRunResult result;

    assert(gait_controller_dry_run(&command, &result));
    assert(result.raw_command_valid);
    assert(result.mode == GAIT_MODE_STAND);
    assert_targets_valid(&result);
}

static void test_translate_command()
{
    MotionCommand command = motion_command_zero();
    command.velocity_x_mps = 0.05f;
    GaitDryRunResult result;

    assert(gait_controller_dry_run(&command, &result));
    assert(result.raw_command_valid);
    assert(result.mode == GAIT_MODE_TRANSLATE);
    assert(result.bounded_command.velocity_x_mps == 0.05f);
    assert_targets_valid(&result);
}

static void test_combined_command_is_bounded()
{
    MotionCommand command;
    command.velocity_x_mps = 5.0f;
    command.velocity_y_mps = -5.0f;
    command.yaw_rate_radps = 5.0f;
    GaitDryRunResult result;
    MotionCommandLimits limits = motion_command_default_limits();

    assert(gait_controller_dry_run(&command, &result));
    assert(result.raw_command_valid);
    assert(result.mode == GAIT_MODE_COMBINED);
    assert(result.bounded_command.velocity_x_mps == limits.max_velocity_x_mps);
    assert(result.bounded_command.velocity_y_mps == -limits.max_velocity_y_mps);
    assert(result.bounded_command.yaw_rate_radps == limits.max_yaw_rate_radps);
    assert_targets_valid(&result);
}

int main()
{
    test_zero_command_stands();
    test_translate_command();
    test_combined_command_is_bounded();
    return 0;
}
