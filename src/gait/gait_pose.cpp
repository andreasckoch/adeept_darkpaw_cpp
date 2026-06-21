#include "gait_pose.h"

#include <sstream>
#include <vector>

bool gait_read_text_file(const std::string &path, std::string *content, std::string *error);
bool gait_json_get_string(const std::string &json, const std::string &field, std::string *value);
bool gait_json_get_int(const std::string &json, const std::string &field, int *value);
std::vector<std::string> gait_json_get_object_array(const std::string &json, const std::string &field);

void gait_pose_init(GaitPose *pose)
{
    pose->name.clear();
    pose->description.clear();
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        pose->pulse_microsec[i] = 0;
        pose->channel_present[i] = false;
    }
}

bool gait_pose_validate(const GaitPose &pose, std::string *error)
{
    if (pose.name.empty())
    {
        if (error != 0) { *error = "pose is missing name"; }
        return false;
    }

    for (int channel = 0; channel < SERVO_COUNT; channel++)
    {
        if (!pose.channel_present[channel])
        {
            std::stringstream message;
            message << "pose '" << pose.name << "' is missing channel " << channel;
            if (error != 0) { *error = message.str(); }
            return false;
        }

        ServoPulseLimits limits;
        servo_get_pulse_limits(channel, &limits);
        int pulse = pose.pulse_microsec[channel];
        if (pulse < limits.min_microsec || pulse > limits.max_microsec)
        {
            std::stringstream message;
            message << "pose '" << pose.name << "' channel " << channel
                    << " pulse " << pulse << " us is outside ["
                    << limits.min_microsec << ", " << limits.max_microsec << "]";
            if (error != 0) { *error = message.str(); }
            return false;
        }
    }

    return true;
}

bool gait_pose_parse_json(const std::string &json, GaitPose *pose, std::string *error)
{
    gait_pose_init(pose);
    if (!gait_json_get_string(json, "name", &pose->name))
    {
        if (error != 0) { *error = "pose JSON is missing string field 'name'"; }
        return false;
    }
    gait_json_get_string(json, "description", &pose->description);

    std::vector<std::string> servos = gait_json_get_object_array(json, "servos");
    if (servos.empty())
    {
        if (error != 0) { *error = "pose JSON is missing non-empty 'servos' array"; }
        return false;
    }

    for (size_t i = 0; i < servos.size(); i++)
    {
        int channel = -1;
        int pulse = 0;
        if (!gait_json_get_int(servos[i], "channel", &channel) ||
            !gait_json_get_int(servos[i], "pulse_microsec", &pulse))
        {
            if (error != 0) { *error = "servo entry must contain channel and pulse_microsec"; }
            return false;
        }
        if (!servo_is_valid_index(channel))
        {
            std::stringstream message;
            message << "invalid servo channel " << channel;
            if (error != 0) { *error = message.str(); }
            return false;
        }
        if (pose->channel_present[channel])
        {
            std::stringstream message;
            message << "duplicate servo channel " << channel;
            if (error != 0) { *error = message.str(); }
            return false;
        }

        pose->channel_present[channel] = true;
        pose->pulse_microsec[channel] = pulse;
    }

    return gait_pose_validate(*pose, error);
}

bool gait_pose_load_json(const std::string &path, GaitPose *pose, std::string *error)
{
    std::string content;
    if (!gait_read_text_file(path, &content, error))
    {
        return false;
    }

    return gait_pose_parse_json(content, pose, error);
}
