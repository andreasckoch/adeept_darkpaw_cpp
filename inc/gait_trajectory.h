#ifndef GAIT_TRAJECTORY_H_
#define GAIT_TRAJECTORY_H_

#include "gait_definition.h"
#include "gait_pose.h"

#include <string>
#include <vector>

struct GaitTrajectorySample
{
    int timestamp_ms;
    std::string phase;
    int step;
    int channel;
    int pulse_microsec;
    int ticks;
};

bool gait_compile_trajectory(const GaitDefinition &definition,
                             const std::string &poses_dir,
                             int max_delta_microsec,
                             std::vector<GaitTrajectorySample> *samples,
                             std::string *error);

bool gait_write_trajectory_csv(const std::string &path,
                               const std::vector<GaitTrajectorySample> &samples,
                               std::string *error);

bool gait_read_trajectory_csv(const std::string &path,
                              std::vector<GaitTrajectorySample> *samples,
                              std::string *error);

bool gait_validate_trajectory(const std::vector<GaitTrajectorySample> &samples,
                              int max_delta_microsec,
                              std::string *error);

#endif /* GAIT_TRAJECTORY_H_ */
