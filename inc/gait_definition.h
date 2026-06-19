#ifndef GAIT_DEFINITION_H_
#define GAIT_DEFINITION_H_

#include <string>
#include <vector>

struct GaitPhaseDefinition
{
    std::string name;
    std::string from_pose;
    std::string to_pose;
    int duration_ms;
    int steps;
};

struct GaitDefinition
{
    std::string name;
    std::string description;
    std::vector<GaitPhaseDefinition> phases;
};

void gait_definition_init(GaitDefinition *definition);
bool gait_definition_load_json(const std::string &path, GaitDefinition *definition, std::string *error);
bool gait_definition_parse_json(const std::string &json, GaitDefinition *definition, std::string *error);
bool gait_definition_validate(const GaitDefinition &definition, std::string *error);

#endif /* GAIT_DEFINITION_H_ */
