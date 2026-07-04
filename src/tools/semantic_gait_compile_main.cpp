#include "semantic_gait.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void print_usage(const char *program)
{
    printf("Usage: %s --profile FILE.json --poses DIR --gait FILE.json --output FILE.csv [--max-delta-us N]\n", program);
}

int main(int argc, char **argv)
{
    const char *profile_path = 0;
    const char *poses_dir = 0;
    const char *gait_path = 0;
    const char *output_path = 0;
    int max_delta_microsec = 80;
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
        else if (strcmp(argv[i], "--gait") == 0 && i + 1 < argc)
        {
            gait_path = argv[++i];
        }
        else if (strcmp(argv[i], "--output") == 0 && i + 1 < argc)
        {
            output_path = argv[++i];
        }
        else if (strcmp(argv[i], "--max-delta-us") == 0 && i + 1 < argc)
        {
            max_delta_microsec = atoi(argv[++i]);
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

    if (profile_path == 0 || poses_dir == 0 || gait_path == 0 || output_path == 0 || max_delta_microsec <= 0)
    {
        print_usage(argv[0]);
        return 2;
    }

    SemanticRobotProfile profile;
    SemanticGaitDefinition definition;
    std::vector<GaitTrajectorySample> samples;
    std::string error;
    if (!semantic_profile_load_json(profile_path, &profile, &error) ||
        !semantic_gait_load_json(gait_path, &definition, &error) ||
        !semantic_gait_compile_trajectory(definition, profile, poses_dir, max_delta_microsec, &samples, &error) ||
        !gait_write_trajectory_csv(output_path, samples, &error))
    {
        fprintf(stderr, "Failed to compile semantic gait: %s\n", error.c_str());
        return 1;
    }

    printf("Compiled semantic gait '%s' to %s (%lu samples, %lu frames)\n",
           definition.name.c_str(),
           output_path,
           (unsigned long)samples.size(),
           (unsigned long)(samples.size() / SERVO_COUNT));
    return 0;
}
