#include "motion_command.h"

#include <math.h>

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float apply_deadzone(float value, float deadzone)
{
    if (fabs(value) < deadzone)
    {
        return 0.0f;
    }
    return value;
}

MotionCommand motion_command_zero()
{
    MotionCommand command;
    command.velocity_x_mps = 0.0f;
    command.velocity_y_mps = 0.0f;
    command.yaw_rate_radps = 0.0f;
    return command;
}

MotionCommandLimits motion_command_default_limits()
{
    MotionCommandLimits limits;
    limits.max_velocity_x_mps = 0.10f;
    limits.max_velocity_y_mps = 0.08f;
    limits.max_yaw_rate_radps = 0.80f;
    limits.deadzone = 0.001f;
    return limits;
}

bool motion_command_is_finite(const MotionCommand *command)
{
    return command != 0 &&
           isfinite(command->velocity_x_mps) &&
           isfinite(command->velocity_y_mps) &&
           isfinite(command->yaw_rate_radps);
}

MotionCommand motion_command_sanitize(const MotionCommand *command, const MotionCommandLimits *limits, bool *input_was_valid)
{
    MotionCommandLimits active_limits = limits != 0 ? *limits : motion_command_default_limits();
    bool valid = motion_command_is_finite(command);
    MotionCommand sanitized = valid ? *command : motion_command_zero();

    sanitized.velocity_x_mps = apply_deadzone(sanitized.velocity_x_mps, active_limits.deadzone);
    sanitized.velocity_y_mps = apply_deadzone(sanitized.velocity_y_mps, active_limits.deadzone);
    sanitized.yaw_rate_radps = apply_deadzone(sanitized.yaw_rate_radps, active_limits.deadzone);

    sanitized.velocity_x_mps = clamp_float(sanitized.velocity_x_mps,
                                           -active_limits.max_velocity_x_mps,
                                           active_limits.max_velocity_x_mps);
    sanitized.velocity_y_mps = clamp_float(sanitized.velocity_y_mps,
                                           -active_limits.max_velocity_y_mps,
                                           active_limits.max_velocity_y_mps);
    sanitized.yaw_rate_radps = clamp_float(sanitized.yaw_rate_radps,
                                           -active_limits.max_yaw_rate_radps,
                                           active_limits.max_yaw_rate_radps);

    if (input_was_valid != 0)
    {
        *input_was_valid = valid;
    }

    return sanitized;
}
