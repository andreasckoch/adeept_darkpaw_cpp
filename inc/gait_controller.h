#ifndef GAIT_CONTROLLER_H_
#define GAIT_CONTROLLER_H_

#include "motion_command.h"
#include "servo_calibration.h"

enum GaitMode
{
    GAIT_MODE_STAND = 0,
    GAIT_MODE_TRANSLATE = 1,
    GAIT_MODE_ROTATE = 2,
    GAIT_MODE_COMBINED = 3,
};

struct ServoDryRunTarget
{
    int channel;
    int pulse_microsec;
    int ticks;
    bool within_limits;
};

struct GaitDryRunResult
{
    MotionCommand raw_command;
    MotionCommand bounded_command;
    bool raw_command_valid;
    GaitMode mode;
    ServoDryRunTarget targets[SERVO_COUNT];
};

const char *gait_mode_name(GaitMode mode);
bool gait_controller_dry_run(const MotionCommand *command, GaitDryRunResult *result);

#endif /* GAIT_CONTROLLER_H_ */
