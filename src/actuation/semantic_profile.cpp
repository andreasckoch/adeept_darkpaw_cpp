#include "semantic_profile.h"

#include "servo_calibration.h"

#include <sstream>
#include <vector>

bool gait_read_text_file(const std::string &path, std::string *content, std::string *error);
bool gait_json_get_string(const std::string &json, const std::string &field, std::string *value);
bool gait_json_get_int(const std::string &json, const std::string &field, int *value);
std::vector<std::string> gait_json_get_object_array(const std::string &json, const std::string &field);

static const char *FORE_AFT_POSITIONS[] = { "back", "neutral", "front" };
static const char *LIFT_POSITIONS[] = { "down", "neutral", "up" };
static const char *STANCE_POSITIONS[] = { "close", "neutral", "wide" };

static const char *const *expected_positions_for_axis(const std::string &axis)
{
    if (axis == "fore_aft")
    {
        return FORE_AFT_POSITIONS;
    }
    if (axis == "lift")
    {
        return LIFT_POSITIONS;
    }
    if (axis == "stance")
    {
        return STANCE_POSITIONS;
    }
    return 0;
}

static bool has_name(const std::vector<SemanticPosition> &positions, const char *name)
{
    for (size_t i = 0; i < positions.size(); i++)
    {
        if (positions[i].name == name)
        {
            return true;
        }
    }
    return false;
}

void semantic_profile_init(SemanticRobotProfile *profile)
{
    profile->name.clear();
    profile->description.clear();
    profile->legs.clear();
}

bool semantic_profile_validate(const SemanticRobotProfile &profile, std::string *error)
{
    if (profile.name.empty())
    {
        if (error != 0) { *error = "semantic profile is missing name"; }
        return false;
    }
    if (profile.legs.size() != 4)
    {
        if (error != 0) { *error = "semantic profile must define exactly four legs"; }
        return false;
    }

    bool channels_seen[SERVO_COUNT] = { false };
    for (size_t leg_index = 0; leg_index < profile.legs.size(); leg_index++)
    {
        const SemanticLeg &leg = profile.legs[leg_index];
        if (leg.name.empty() || leg.joints.size() != 3)
        {
            if (error != 0) { *error = "each semantic leg must have a name and exactly three joints"; }
            return false;
        }
        for (size_t other_leg = 0; other_leg < leg_index; other_leg++)
        {
            if (profile.legs[other_leg].name == leg.name)
            {
                if (error != 0) { *error = "semantic profile contains duplicate leg names"; }
                return false;
            }
        }

        for (size_t joint_index = 0; joint_index < leg.joints.size(); joint_index++)
        {
            const SemanticJoint &joint = leg.joints[joint_index];
            const char *const *expected_positions = expected_positions_for_axis(joint.axis);
            if (expected_positions == 0 || !servo_is_valid_index(joint.channel) || channels_seen[joint.channel])
            {
                if (error != 0) { *error = "semantic profile has an invalid, duplicate, or unsupported joint"; }
                return false;
            }
            channels_seen[joint.channel] = true;
            if (joint.positions.size() != 3)
            {
                if (error != 0) { *error = "each semantic joint must define exactly three positions"; }
                return false;
            }
            for (size_t other_joint = 0; other_joint < joint_index; other_joint++)
            {
                if (leg.joints[other_joint].axis == joint.axis)
                {
                    if (error != 0) { *error = "a semantic leg contains duplicate joint axes"; }
                    return false;
                }
            }

            ServoPulseLimits limits;
            servo_get_pulse_limits(joint.channel, &limits);
            for (size_t position_index = 0; position_index < joint.positions.size(); position_index++)
            {
                const SemanticPosition &position = joint.positions[position_index];
                if (!has_name(joint.positions, expected_positions[position_index]) ||
                    position.pulse_microsec < limits.min_microsec ||
                    position.pulse_microsec > limits.max_microsec)
                {
                    std::stringstream message;
                    message << "semantic joint '" << leg.name << "." << joint.axis
                            << "' has missing positions or a pulse outside channel calibration";
                    if (error != 0) { *error = message.str(); }
                    return false;
                }
                for (size_t other_position = 0; other_position < position_index; other_position++)
                {
                    if (joint.positions[other_position].name == position.name)
                    {
                        if (error != 0) { *error = "a semantic joint contains duplicate position names"; }
                        return false;
                    }
                }
            }
        }
    }

    for (int channel = 0; channel < SERVO_COUNT; channel++)
    {
        if (!channels_seen[channel])
        {
            if (error != 0) { *error = "semantic profile does not map every servo channel"; }
            return false;
        }
    }
    return true;
}

bool semantic_profile_parse_json(const std::string &json, SemanticRobotProfile *profile, std::string *error)
{
    if (profile == 0)
    {
        if (error != 0) { *error = "semantic profile output is null"; }
        return false;
    }

    semantic_profile_init(profile);
    if (!gait_json_get_string(json, "name", &profile->name))
    {
        if (error != 0) { *error = "semantic profile JSON is missing string field 'name'"; }
        return false;
    }
    gait_json_get_string(json, "description", &profile->description);

    std::vector<std::string> legs = gait_json_get_object_array(json, "legs");
    if (legs.empty())
    {
        if (error != 0) { *error = "semantic profile JSON is missing non-empty 'legs' array"; }
        return false;
    }

    for (size_t leg_index = 0; leg_index < legs.size(); leg_index++)
    {
        SemanticLeg leg;
        if (!gait_json_get_string(legs[leg_index], "name", &leg.name))
        {
            if (error != 0) { *error = "semantic leg is missing string field 'name'"; }
            return false;
        }

        std::vector<std::string> joints = gait_json_get_object_array(legs[leg_index], "joints");
        for (size_t joint_index = 0; joint_index < joints.size(); joint_index++)
        {
            SemanticJoint joint;
            if (!gait_json_get_string(joints[joint_index], "axis", &joint.axis) ||
                !gait_json_get_int(joints[joint_index], "channel", &joint.channel))
            {
                if (error != 0) { *error = "semantic joint must contain axis and channel"; }
                return false;
            }

            std::vector<std::string> positions = gait_json_get_object_array(joints[joint_index], "positions");
            for (size_t position_index = 0; position_index < positions.size(); position_index++)
            {
                SemanticPosition position;
                if (!gait_json_get_string(positions[position_index], "name", &position.name) ||
                    !gait_json_get_int(positions[position_index], "pulse_microsec", &position.pulse_microsec))
                {
                    if (error != 0) { *error = "semantic position must contain name and pulse_microsec"; }
                    return false;
                }
                joint.positions.push_back(position);
            }
            leg.joints.push_back(joint);
        }
        profile->legs.push_back(leg);
    }

    return semantic_profile_validate(*profile, error);
}

bool semantic_profile_load_json(const std::string &path, SemanticRobotProfile *profile, std::string *error)
{
    std::string content;
    if (!gait_read_text_file(path, &content, error))
    {
        return false;
    }
    return semantic_profile_parse_json(content, profile, error);
}

bool semantic_profile_get_pulse(const SemanticRobotProfile &profile,
                                const std::string &leg_name,
                                const std::string &axis,
                                const std::string &position_name,
                                int *channel,
                                int *pulse_microsec,
                                std::string *error)
{
    for (size_t leg_index = 0; leg_index < profile.legs.size(); leg_index++)
    {
        const SemanticLeg &leg = profile.legs[leg_index];
        if (leg.name != leg_name)
        {
            continue;
        }
        for (size_t joint_index = 0; joint_index < leg.joints.size(); joint_index++)
        {
            const SemanticJoint &joint = leg.joints[joint_index];
            if (joint.axis != axis)
            {
                continue;
            }
            for (size_t position_index = 0; position_index < joint.positions.size(); position_index++)
            {
                const SemanticPosition &position = joint.positions[position_index];
                if (position.name == position_name)
                {
                    if (channel != 0) { *channel = joint.channel; }
                    if (pulse_microsec != 0) { *pulse_microsec = position.pulse_microsec; }
                    return true;
                }
            }
            break;
        }
        break;
    }

    std::stringstream message;
    message << "unknown semantic target '" << leg_name << "." << axis << "." << position_name << "'";
    if (error != 0) { *error = message.str(); }
    return false;
}
