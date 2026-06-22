#include "semantic_gait.h"

#include <sstream>
#include <vector>

bool gait_read_text_file(const std::string &path, std::string *content, std::string *error);
bool gait_json_get_string(const std::string &json, const std::string &field, std::string *value);
bool gait_json_get_int(const std::string &json, const std::string &field, int *value);
std::vector<std::string> gait_json_get_object_array(const std::string &json, const std::string &field);

static bool valid_name(const std::string &name)
{
    if (name.empty())
    {
        return false;
    }
    for (size_t i = 0; i < name.size(); i++)
    {
        char c = name[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '_' || c == '-'))
        {
            return false;
        }
    }
    return true;
}

void semantic_gait_init(SemanticGaitDefinition *definition)
{
    definition->name.clear();
    definition->description.clear();
    definition->initial_pose.clear();
    definition->phases.clear();
}

bool semantic_gait_validate(const SemanticGaitDefinition &definition, std::string *error)
{
    if (!valid_name(definition.name) || !valid_name(definition.initial_pose) || definition.phases.empty())
    {
        if (error != 0) { *error = "semantic gait requires a safe name, initial_pose, and at least one phase"; }
        return false;
    }
    for (size_t i = 0; i < definition.phases.size(); i++)
    {
        const SemanticGaitPhase &phase = definition.phases[i];
        bool has_pose = !phase.target_pose.empty();
        bool has_targets = !phase.targets.empty();
        if (phase.name.empty() || phase.duration_ms <= 0 || phase.steps <= 0 || has_pose == has_targets)
        {
            if (error != 0) { *error = "each semantic gait phase must choose exactly one target pose or target list"; }
            return false;
        }
        if (has_pose && !valid_name(phase.target_pose))
        {
            if (error != 0) { *error = "semantic gait phase target pose is invalid"; }
            return false;
        }
    }
    return true;
}

bool semantic_gait_parse_json(const std::string &json, SemanticGaitDefinition *definition, std::string *error)
{
    if (definition == 0)
    {
        if (error != 0) { *error = "semantic gait output is null"; }
        return false;
    }
    semantic_gait_init(definition);
    if (!gait_json_get_string(json, "name", &definition->name) ||
        !gait_json_get_string(json, "initial_pose", &definition->initial_pose))
    {
        if (error != 0) { *error = "semantic gait must contain name and initial_pose"; }
        return false;
    }
    gait_json_get_string(json, "description", &definition->description);

    std::vector<std::string> phases = gait_json_get_object_array(json, "phases");
    for (size_t i = 0; i < phases.size(); i++)
    {
        SemanticGaitPhase phase;
        if (!gait_json_get_string(phases[i], "name", &phase.name) ||
            !gait_json_get_int(phases[i], "duration_ms", &phase.duration_ms) ||
            !gait_json_get_int(phases[i], "steps", &phase.steps))
        {
            if (error != 0) { *error = "semantic gait phase must contain name, duration_ms, and steps"; }
            return false;
        }
        gait_json_get_string(phases[i], "pose", &phase.target_pose);
        std::vector<std::string> targets = gait_json_get_object_array(phases[i], "targets");
        for (size_t target_index = 0; target_index < targets.size(); target_index++)
        {
            SemanticTarget target;
            if (!semantic_target_parse_json(targets[target_index], &target, error))
            {
                return false;
            }
            phase.targets.push_back(target);
        }
        definition->phases.push_back(phase);
    }
    return semantic_gait_validate(*definition, error);
}

bool semantic_gait_load_json(const std::string &path, SemanticGaitDefinition *definition, std::string *error)
{
    std::string content;
    if (!gait_read_text_file(path, &content, error))
    {
        return false;
    }
    return semantic_gait_parse_json(content, definition, error);
}

bool semantic_gait_compile_trajectory(const SemanticGaitDefinition &definition,
                                      const SemanticRobotProfile &profile,
                                      const std::string &poses_dir,
                                      int max_delta_microsec,
                                      std::vector<GaitTrajectorySample> *samples,
                                      std::string *error)
{
    if (samples == 0)
    {
        if (error != 0) { *error = "semantic trajectory samples output is null"; }
        return false;
    }
    samples->clear();
    if (!semantic_profile_validate(profile, error) || !semantic_gait_validate(definition, error))
    {
        return false;
    }

    GaitPose current_pose;
    if (!semantic_pose_resolve(profile, poses_dir, definition.initial_pose, &current_pose, error))
    {
        return false;
    }

    int start_ms = 0;
    for (size_t phase_index = 0; phase_index < definition.phases.size(); phase_index++)
    {
        const SemanticGaitPhase &phase = definition.phases[phase_index];
        GaitPose target_pose;
        if (!phase.target_pose.empty())
        {
            if (!semantic_pose_resolve(profile, poses_dir, phase.target_pose, &target_pose, error))
            {
                return false;
            }
        }
        else
        {
            target_pose = current_pose;
            if (!semantic_apply_targets(profile, phase.targets, &target_pose, error) ||
                !gait_pose_validate(target_pose, error))
            {
                return false;
            }
        }

        if (!gait_append_pose_transition(current_pose,
                                         target_pose,
                                         phase.name,
                                         phase.duration_ms,
                                         phase.steps,
                                         start_ms,
                                         max_delta_microsec,
                                         samples,
                                         error))
        {
            return false;
        }
        start_ms += phase.duration_ms;
        current_pose = target_pose;
    }
    return gait_validate_trajectory(*samples, max_delta_microsec, error);
}
