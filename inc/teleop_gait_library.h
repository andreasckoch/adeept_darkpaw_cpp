#ifndef TELEOP_GAIT_LIBRARY_H_
#define TELEOP_GAIT_LIBRARY_H_

#include "gait_trajectory.h"
#include "semantic_gait.h"
#include "teleop_message.h"

#include <string>
#include <vector>

struct TeleopGaitTiming
{
    int phase_duration_ms;
    int phase_steps;
    int max_delta_microsec;
};

TeleopGaitTiming teleop_gait_default_timing();
bool teleop_gait_build_definition(TeleopMovementType movement,
                                  const TeleopGaitTiming &timing,
                                  SemanticGaitDefinition *definition,
                                  std::string *error);
bool teleop_gait_compile_loop(TeleopMovementType movement,
                              const SemanticRobotProfile &profile,
                              const std::string &poses_dir,
                              const TeleopGaitTiming &timing,
                              std::vector<GaitTrajectorySample> *samples,
                              std::string *error);

#endif /* TELEOP_GAIT_LIBRARY_H_ */
