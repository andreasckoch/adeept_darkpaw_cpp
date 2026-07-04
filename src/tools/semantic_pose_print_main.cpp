#include "semantic_pose.h"

#include <stdio.h>
#include <string.h>

static void print_usage(const char *program)
{
    printf("Usage: %s --profile FILE.json --poses DIR --pose NAME\n", program);
    printf("Resolves a semantic pose without opening pigpio, I2C, or commanding servos.\n");
}

int main(int argc, char **argv)
{
    const char *profile_path = 0;
    const char *poses_dir = 0;
    const char *pose_name = 0;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--profile") == 0 && i + 1 < argc)
        {
            profile_path = argv[++i];
        }
        else if (strcmp(argv[i], "--poses") == 0 && i + 1 < argc)
        {
            poses_dir = argv[++i];
        }
        else if (strcmp(argv[i], "--pose") == 0 && i + 1 < argc)
        {
            pose_name = argv[++i];
        }
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            print_usage(argv[0]);
            return 0;
        }
        else
        {
            print_usage(argv[0]);
            return 2;
        }
    }

    if (profile_path == 0 || poses_dir == 0 || pose_name == 0)
    {
        print_usage(argv[0]);
        return 2;
    }

    SemanticRobotProfile profile;
    GaitPose pose;
    std::string error;
    if (!semantic_profile_load_json(profile_path, &profile, &error) ||
        !semantic_pose_resolve(profile, poses_dir, pose_name, &pose, &error))
    {
        fprintf(stderr, "Failed to resolve semantic pose: %s\n", error.c_str());
        return 1;
    }

    printf("Semantic pose '%s'\n", pose.name.c_str());
    for (size_t leg_index = 0; leg_index < profile.legs.size(); leg_index++)
    {
        const SemanticLeg &leg = profile.legs[leg_index];
        printf("  %s\n", leg.name.c_str());
        for (size_t joint_index = 0; joint_index < leg.joints.size(); joint_index++)
        {
            const SemanticJoint &joint = leg.joints[joint_index];
            printf("    %-9s channel=%2d pulse=%4d us\n",
                   joint.axis.c_str(), joint.channel, pose.pulse_microsec[joint.channel]);
        }
    }
    return 0;
}
