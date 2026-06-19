#include "gait_definition.h"
#include "gait_trajectory.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct ValidateOptions
{
    std::string gait_path;
    std::string poses_dir;
    int max_delta_microsec;
};

static void print_usage(const char *program)
{
    printf("Usage: %s --gait FILE --poses DIR [--max-delta-us N]\n", program);
}

static bool parse_args(int argc, char **argv, ValidateOptions *options)
{
    options->max_delta_microsec = 200;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--gait") == 0 && i + 1 < argc)
        {
            options->gait_path = argv[++i];
        }
        else if (strcmp(argv[i], "--poses") == 0 && i + 1 < argc)
        {
            options->poses_dir = argv[++i];
        }
        else if (strcmp(argv[i], "--max-delta-us") == 0 && i + 1 < argc)
        {
            options->max_delta_microsec = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            print_usage(argv[0]);
            exit(0);
        }
        else
        {
            return false;
        }
    }

    return !options->gait_path.empty() && !options->poses_dir.empty();
}

int main(int argc, char **argv)
{
    ValidateOptions options;
    if (!parse_args(argc, argv, &options))
    {
        print_usage(argv[0]);
        return 2;
    }

    GaitDefinition definition;
    std::string error;
    if (!gait_definition_load_json(options.gait_path, &definition, &error))
    {
        fprintf(stderr, "Gait definition invalid: %s\n", error.c_str());
        return 1;
    }

    std::vector<GaitTrajectorySample> samples;
    if (!gait_compile_trajectory(definition, options.poses_dir, options.max_delta_microsec, &samples, &error))
    {
        fprintf(stderr, "Gait validation failed: %s\n", error.c_str());
        return 1;
    }

    printf("Gait '%s' is valid: %lu samples, %lu frames\n",
           definition.name.c_str(),
           (unsigned long)samples.size(),
           (unsigned long)(samples.size() / SERVO_COUNT));
    return 0;
}
