#include "gait_controller.h"

#include "pca9685.h"

#include <math.h>

static float safe_divide(float value, float divisor)
{
    if (divisor == 0.0f)
    {
        return 0.0f;
    }
    return value / divisor;
}

static int round_to_int(float value)
{
    if (value >= 0.0f)
    {
        return (int)(value + 0.5f);
    }
    return (int)(value - 0.5f);
}

static int neutral_pulse_for_servo(int channel)
{
    ServoPulseLimits limits;
    if (!servo_get_pulse_limits(channel, &limits))
    {
        return 1500;
    }

    return (limits.min_microsec + limits.max_microsec) / 2;
}

static int pulse_range_for_servo(int channel)
{
    ServoPulseLimits limits;
    if (!servo_get_pulse_limits(channel, &limits))
    {
        return 0;
    }

    return limits.max_microsec - limits.min_microsec;
}

static GaitMode select_mode(const MotionCommand *command)
{
    bool translating = fabs(command->velocity_x_mps) > 0.0f || fabs(command->velocity_y_mps) > 0.0f;
    bool rotating = fabs(command->yaw_rate_radps) > 0.0f;

    if (translating && rotating)
    {
        return GAIT_MODE_COMBINED;
    }
    if (translating)
    {
        return GAIT_MODE_TRANSLATE;
    }
    if (rotating)
    {
        return GAIT_MODE_ROTATE;
    }
    return GAIT_MODE_STAND;
}

static float normalized_servo_intent(int channel, const MotionCommand *command, const MotionCommandLimits *limits)
{
    /*
     * Placeholder dry-run phase weights.
     *
     * These values are not measured from the robot, looked up from a gait
     * reference, or derived from inverse kinematics. They only provide a
     * deterministic, bounded signal so the dry-run command pipeline can be
     * tested end to end without moving hardware. Replace them with calibrated
     * geometry, a real gait generator, or simulator-derived values before using
     * this path for actual walking.
     */
    static const float stride_phase[SERVO_COUNT] = {
         1.0f,  0.7f, -0.7f,
        -1.0f, -0.7f,  0.7f,
        -1.0f,  0.7f, -0.7f,
         1.0f, -0.7f,  0.7f,
    };

    static const float lateral_phase[SERVO_COUNT] = {
         0.5f, -0.3f,  0.3f,
         0.5f,  0.3f, -0.3f,
        -0.5f, -0.3f,  0.3f,
        -0.5f,  0.3f, -0.3f,
    };

    static const float yaw_phase[SERVO_COUNT] = {
         0.4f,  0.2f, -0.2f,
        -0.4f, -0.2f,  0.2f,
         0.4f, -0.2f,  0.2f,
        -0.4f,  0.2f, -0.2f,
    };

    float vx = safe_divide(command->velocity_x_mps, limits->max_velocity_x_mps);
    float vy = safe_divide(command->velocity_y_mps, limits->max_velocity_y_mps);
    float yaw = safe_divide(command->yaw_rate_radps, limits->max_yaw_rate_radps);

    float intent = (0.55f * vx * stride_phase[channel]) +
                   (0.25f * vy * lateral_phase[channel]) +
                   (0.20f * yaw * yaw_phase[channel]);

    if (intent > 1.0f)
    {
        return 1.0f;
    }
    if (intent < -1.0f)
    {
        return -1.0f;
    }
    return intent;
}

const char *gait_mode_name(GaitMode mode)
{
    switch (mode)
    {
        case GAIT_MODE_STAND:
            return "stand";
        case GAIT_MODE_TRANSLATE:
            return "translate";
        case GAIT_MODE_ROTATE:
            return "rotate";
        case GAIT_MODE_COMBINED:
            return "combined";
        default:
            return "unknown";
    }
}

bool gait_controller_dry_run(const MotionCommand *command, GaitDryRunResult *result)
{
    if (result == 0)
    {
        return false;
    }

    MotionCommandLimits limits = motion_command_default_limits();
    result->raw_command = command != 0 ? *command : motion_command_zero();
    result->bounded_command = motion_command_sanitize(command, &limits, &result->raw_command_valid);
    result->mode = select_mode(&result->bounded_command);

    for (int channel = 0; channel < SERVO_COUNT; channel++)
    {
        int neutral_pulse = neutral_pulse_for_servo(channel);
        int range = pulse_range_for_servo(channel);
        int pulse_microsec = neutral_pulse;

        if (result->mode != GAIT_MODE_STAND)
        {
            float intent = normalized_servo_intent(channel, &result->bounded_command, &limits);
            pulse_microsec += round_to_int(intent * range * 0.15f);
        }

        int clamped_pulse = servo_clamp_pulse_microseconds(channel, pulse_microsec);

        result->targets[channel].channel = channel;
        result->targets[channel].pulse_microsec = clamped_pulse;
        result->targets[channel].ticks = pca9685_pulse_microseconds_to_ticks(clamped_pulse);
        result->targets[channel].within_limits = clamped_pulse == pulse_microsec;
    }

    return true;
}
