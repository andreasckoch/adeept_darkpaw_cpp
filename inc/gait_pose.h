#ifndef GAIT_POSE_H_
#define GAIT_POSE_H_

#include "servo_calibration.h"

#include <string>

struct GaitPose
{
    std::string name;
    std::string description;
    int pulse_microsec[SERVO_COUNT];
    bool channel_present[SERVO_COUNT];
};

void gait_pose_init(GaitPose *pose);
bool gait_pose_load_json(const std::string &path, GaitPose *pose, std::string *error);
bool gait_pose_parse_json(const std::string &json, GaitPose *pose, std::string *error);
bool gait_pose_validate(const GaitPose &pose, std::string *error);

#endif /* GAIT_POSE_H_ */
