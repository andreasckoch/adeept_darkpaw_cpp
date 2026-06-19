#include "gait_definition.h"

#include <sstream>
#include <vector>

bool gait_read_text_file(const std::string &path, std::string *content, std::string *error);
bool gait_json_get_string(const std::string &json, const std::string &field, std::string *value);
bool gait_json_get_int(const std::string &json, const std::string &field, int *value);
std::vector<std::string> gait_json_get_object_array(const std::string &json, const std::string &field);

void gait_definition_init(GaitDefinition *definition)
{
    definition->name.clear();
    definition->description.clear();
    definition->phases.clear();
}

bool gait_definition_validate(const GaitDefinition &definition, std::string *error)
{
    if (definition.name.empty())
    {
        if (error != 0) { *error = "gait definition is missing name"; }
        return false;
    }
    if (definition.phases.empty())
    {
        if (error != 0) { *error = "gait definition has no phases"; }
        return false;
    }

    for (size_t i = 0; i < definition.phases.size(); i++)
    {
        const GaitPhaseDefinition &phase = definition.phases[i];
        if (phase.name.empty() || phase.from_pose.empty() || phase.to_pose.empty())
        {
            if (error != 0) { *error = "phase is missing name/from/to"; }
            return false;
        }
        if (phase.duration_ms <= 0)
        {
            std::stringstream message;
            message << "phase '" << phase.name << "' duration_ms must be > 0";
            if (error != 0) { *error = message.str(); }
            return false;
        }
        if (phase.steps <= 0)
        {
            std::stringstream message;
            message << "phase '" << phase.name << "' steps must be > 0";
            if (error != 0) { *error = message.str(); }
            return false;
        }
    }

    return true;
}

bool gait_definition_parse_json(const std::string &json, GaitDefinition *definition, std::string *error)
{
    gait_definition_init(definition);
    if (!gait_json_get_string(json, "name", &definition->name))
    {
        if (error != 0) { *error = "gait JSON is missing string field 'name'"; }
        return false;
    }
    gait_json_get_string(json, "description", &definition->description);

    std::vector<std::string> phases = gait_json_get_object_array(json, "phases");
    if (phases.empty())
    {
        if (error != 0) { *error = "gait JSON is missing non-empty 'phases' array"; }
        return false;
    }

    for (size_t i = 0; i < phases.size(); i++)
    {
        GaitPhaseDefinition phase;
        if (!gait_json_get_string(phases[i], "name", &phase.name) ||
            !gait_json_get_string(phases[i], "from", &phase.from_pose) ||
            !gait_json_get_string(phases[i], "to", &phase.to_pose) ||
            !gait_json_get_int(phases[i], "duration_ms", &phase.duration_ms) ||
            !gait_json_get_int(phases[i], "steps", &phase.steps))
        {
            if (error != 0) { *error = "phase entry must contain name, from, to, duration_ms, and steps"; }
            return false;
        }

        definition->phases.push_back(phase);
    }

    return gait_definition_validate(*definition, error);
}

bool gait_definition_load_json(const std::string &path, GaitDefinition *definition, std::string *error)
{
    std::string content;
    if (!gait_read_text_file(path, &content, error))
    {
        return false;
    }

    return gait_definition_parse_json(content, definition, error);
}
