#include "semantic_pose.h"

#include <sstream>
#include <vector>

bool gait_read_text_file(const std::string &path, std::string *content, std::string *error);
bool gait_json_get_string(const std::string &json, const std::string &field, std::string *value);
std::vector<std::string> gait_json_get_object_array(const std::string &json, const std::string &field);
std::vector<std::string> gait_json_get_string_array(const std::string &json, const std::string &field);

static std::string join_path(const std::string &dir, const std::string &file)
{
    if (dir.empty() || dir[dir.size() - 1] == '/')
    {
        return dir + file;
    }
    return dir + "/" + file;
}

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

static bool target_is_valid_shape(const SemanticTarget &target, std::string *error)
{
    if (target.legs.empty() || target.axis.empty() || target.position.empty())
    {
        if (error != 0) { *error = "semantic target must contain non-empty legs, axis, and position"; }
        return false;
    }
    for (size_t i = 0; i < target.legs.size(); i++)
    {
        if (target.legs[i].empty())
        {
            if (error != 0) { *error = "semantic target contains an empty leg name"; }
            return false;
        }
        for (size_t other = 0; other < i; other++)
        {
            if (target.legs[other] == target.legs[i])
            {
                if (error != 0) { *error = "semantic target contains duplicate leg names"; }
                return false;
            }
        }
    }
    return true;
}

void semantic_pose_init(SemanticPose *pose)
{
    pose->name.clear();
    pose->description.clear();
    pose->base_pose.clear();
    pose->targets.clear();
}

bool semantic_target_parse_json(const std::string &json, SemanticTarget *target, std::string *error)
{
    if (target == 0)
    {
        if (error != 0) { *error = "semantic target output is null"; }
        return false;
    }
    target->legs = gait_json_get_string_array(json, "legs");
    if (!gait_json_get_string(json, "axis", &target->axis) ||
        !gait_json_get_string(json, "position", &target->position))
    {
        if (error != 0) { *error = "semantic target must contain legs, axis, and position"; }
        return false;
    }
    return target_is_valid_shape(*target, error);
}

bool semantic_pose_parse_json(const std::string &json, SemanticPose *pose, std::string *error)
{
    if (pose == 0)
    {
        if (error != 0) { *error = "semantic pose output is null"; }
        return false;
    }
    semantic_pose_init(pose);
    if (!gait_json_get_string(json, "name", &pose->name) || !valid_name(pose->name))
    {
        if (error != 0) { *error = "semantic pose must contain a safe non-empty name"; }
        return false;
    }
    gait_json_get_string(json, "description", &pose->description);
    gait_json_get_string(json, "base", &pose->base_pose);
    if (!pose->base_pose.empty() && !valid_name(pose->base_pose))
    {
        if (error != 0) { *error = "semantic pose base must be a safe pose name"; }
        return false;
    }

    std::vector<std::string> targets = gait_json_get_object_array(json, "targets");
    for (size_t i = 0; i < targets.size(); i++)
    {
        SemanticTarget target;
        if (!semantic_target_parse_json(targets[i], &target, error))
        {
            return false;
        }
        pose->targets.push_back(target);
    }
    if (pose->targets.empty())
    {
        if (error != 0) { *error = "semantic pose must contain a non-empty targets array"; }
        return false;
    }
    return true;
}

bool semantic_pose_load_json(const std::string &path, SemanticPose *pose, std::string *error)
{
    std::string content;
    if (!gait_read_text_file(path, &content, error))
    {
        return false;
    }
    return semantic_pose_parse_json(content, pose, error);
}

bool semantic_apply_targets(const SemanticRobotProfile &profile,
                            const std::vector<SemanticTarget> &targets,
                            GaitPose *pose,
                            std::string *error)
{
    if (pose == 0)
    {
        if (error != 0) { *error = "semantic target pose output is null"; }
        return false;
    }

    bool assigned[SERVO_COUNT] = { false };
    for (size_t target_index = 0; target_index < targets.size(); target_index++)
    {
        const SemanticTarget &target = targets[target_index];
        for (size_t leg_index = 0; leg_index < target.legs.size(); leg_index++)
        {
            int channel = -1;
            int pulse_microsec = 0;
            if (!semantic_profile_get_pulse(profile,
                                            target.legs[leg_index],
                                            target.axis,
                                            target.position,
                                            &channel,
                                            &pulse_microsec,
                                            error))
            {
                return false;
            }
            if (assigned[channel])
            {
                if (error != 0) { *error = "semantic target assigns the same joint more than once"; }
                return false;
            }
            assigned[channel] = true;
            pose->channel_present[channel] = true;
            pose->pulse_microsec[channel] = pulse_microsec;
        }
    }
    return true;
}

static bool semantic_pose_resolve_internal(const SemanticRobotProfile &profile,
                                           const std::string &poses_dir,
                                           const std::string &pose_name,
                                           std::vector<std::string> *resolution_chain,
                                           GaitPose *resolved_pose,
                                           std::string *error)
{
    for (size_t i = 0; i < resolution_chain->size(); i++)
    {
        if ((*resolution_chain)[i] == pose_name)
        {
            if (error != 0) { *error = "semantic pose base chain contains a cycle"; }
            return false;
        }
    }
    resolution_chain->push_back(pose_name);

    SemanticPose pose;
    if (!semantic_pose_load_json(join_path(poses_dir, pose_name + ".json"), &pose, error))
    {
        resolution_chain->pop_back();
        return false;
    }

    if (pose.name != pose_name)
    {
        if (error != 0) { *error = "semantic pose filename and name field do not match"; }
        resolution_chain->pop_back();
        return false;
    }

    if (pose.base_pose.empty())
    {
        gait_pose_init(resolved_pose);
    }
    else if (!semantic_pose_resolve_internal(profile,
                                              poses_dir,
                                              pose.base_pose,
                                              resolution_chain,
                                              resolved_pose,
                                              error))
    {
        resolution_chain->pop_back();
        return false;
    }

    if (!semantic_apply_targets(profile, pose.targets, resolved_pose, error))
    {
        resolution_chain->pop_back();
        return false;
    }
    resolved_pose->name = pose.name;
    resolved_pose->description = pose.description;
    resolution_chain->pop_back();
    return gait_pose_validate(*resolved_pose, error);
}

bool semantic_pose_resolve(const SemanticRobotProfile &profile,
                           const std::string &poses_dir,
                           const std::string &pose_name,
                           GaitPose *resolved_pose,
                           std::string *error)
{
    if (resolved_pose == 0 || !valid_name(pose_name))
    {
        if (error != 0) { *error = "semantic pose name or output is invalid"; }
        return false;
    }
    std::vector<std::string> resolution_chain;
    return semantic_pose_resolve_internal(profile, poses_dir, pose_name, &resolution_chain, resolved_pose, error);
}
