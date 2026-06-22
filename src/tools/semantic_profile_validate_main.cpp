#include "semantic_profile.h"
#include "servo_calibration.h"

#include <stdio.h>
#include <string.h>

static void print_usage(const char *program)
{
    printf("Usage: %s --profile FILE.json\n", program);
}

int main(int argc, char **argv)
{
    const char *profile_path = 0;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--profile") == 0 && i + 1 < argc)
        {
            profile_path = argv[++i];
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

    if (profile_path == 0)
    {
        print_usage(argv[0]);
        return 2;
    }

    SemanticRobotProfile profile;
    std::string error;
    if (!semantic_profile_load_json(profile_path, &profile, &error))
    {
        fprintf(stderr, "Semantic profile invalid: %s\n", error.c_str());
        return 1;
    }

    printf("Semantic profile '%s' is valid: %lu legs, %d channels\n",
           profile.name.c_str(),
           (unsigned long)profile.legs.size(),
           SERVO_COUNT);
    return 0;
}
