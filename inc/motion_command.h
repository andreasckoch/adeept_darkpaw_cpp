#ifndef MOTION_COMMAND_H_
#define MOTION_COMMAND_H_

#include <stdbool.h>

struct MotionCommand
{
    float velocity_x_mps;
    float velocity_y_mps;
    float yaw_rate_radps;
};

struct MotionCommandLimits
{
    float max_velocity_x_mps;
    float max_velocity_y_mps;
    float max_yaw_rate_radps;
    float deadzone;
};

MotionCommand motion_command_zero();
MotionCommandLimits motion_command_default_limits();
bool motion_command_is_finite(const MotionCommand *command);
MotionCommand motion_command_sanitize(const MotionCommand *command, const MotionCommandLimits *limits, bool *input_was_valid);

#endif /* MOTION_COMMAND_H_ */
