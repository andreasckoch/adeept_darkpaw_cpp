#ifndef SEMANTIC_POSE_H_
#define SEMANTIC_POSE_H_

#include "gait_pose.h"
#include "semantic_profile.h"

#include <string>
#include <vector>

struct SemanticTarget
{
    std::vector<std::string> legs;
    std::string axis;
    std::string position;
};

struct SemanticPose
{
    std::string name;
    std::string description;
    std::string base_pose;
    std::vector<SemanticTarget> targets;
};

void semantic_pose_init(SemanticPose *pose);
bool semantic_pose_load_json(const std::string &path, SemanticPose *pose, std::string *error);
bool semantic_pose_parse_json(const std::string &json, SemanticPose *pose, std::string *error);
bool semantic_target_parse_json(const std::string &json, SemanticTarget *target, std::string *error);
bool semantic_apply_targets(const SemanticRobotProfile &profile,
                            const std::vector<SemanticTarget> &targets,
                            GaitPose *pose,
                            std::string *error);
bool semantic_pose_resolve(const SemanticRobotProfile &profile,
                           const std::string &poses_dir,
                           const std::string &pose_name,
                           GaitPose *resolved_pose,
                           std::string *error);

#endif /* SEMANTIC_POSE_H_ */
