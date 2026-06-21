#ifndef SEMANTIC_PROFILE_H_
#define SEMANTIC_PROFILE_H_

#include <string>
#include <vector>

struct SemanticPosition
{
    std::string name;
    int pulse_microsec;
};

struct SemanticJoint
{
    std::string axis;
    int channel;
    std::vector<SemanticPosition> positions;
};

struct SemanticLeg
{
    std::string name;
    std::vector<SemanticJoint> joints;
};

struct SemanticRobotProfile
{
    std::string name;
    std::string description;
    std::vector<SemanticLeg> legs;
};

void semantic_profile_init(SemanticRobotProfile *profile);
bool semantic_profile_load_json(const std::string &path, SemanticRobotProfile *profile, std::string *error);
bool semantic_profile_parse_json(const std::string &json, SemanticRobotProfile *profile, std::string *error);
bool semantic_profile_validate(const SemanticRobotProfile &profile, std::string *error);

bool semantic_profile_get_pulse(const SemanticRobotProfile &profile,
                                const std::string &leg_name,
                                const std::string &axis,
                                const std::string &position,
                                int *channel,
                                int *pulse_microsec,
                                std::string *error);

#endif /* SEMANTIC_PROFILE_H_ */
