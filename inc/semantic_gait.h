#ifndef SEMANTIC_GAIT_H_
#define SEMANTIC_GAIT_H_

#include "gait_trajectory.h"
#include "semantic_pose.h"

#include <string>
#include <vector>

struct SemanticGaitPhase
{
    std::string name;
    std::string target_pose;
    int duration_ms;
    int steps;
    std::vector<SemanticTarget> targets;
};

struct SemanticGaitDefinition
{
    std::string name;
    std::string description;
    std::string initial_pose;
    std::vector<SemanticGaitPhase> phases;
};

void semantic_gait_init(SemanticGaitDefinition *definition);
bool semantic_gait_load_json(const std::string &path, SemanticGaitDefinition *definition, std::string *error);
bool semantic_gait_parse_json(const std::string &json, SemanticGaitDefinition *definition, std::string *error);
bool semantic_gait_validate(const SemanticGaitDefinition &definition, std::string *error);
bool semantic_gait_compile_trajectory(const SemanticGaitDefinition &definition,
                                      const SemanticRobotProfile &profile,
                                      const std::string &poses_dir,
                                      int max_delta_microsec,
                                      std::vector<GaitTrajectorySample> *samples,
                                      std::string *error);

#endif /* SEMANTIC_GAIT_H_ */
