#include "gait_trajectory.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void print_usage(const char *program)
{
    printf("Usage: %s --trajectory FILE.csv [--max-delta-us N]\n", program);
}

static int frame_max_delta(const std::vector<GaitTrajectorySample> &samples, size_t frame_start)
{
    if (frame_start == 0)
    {
        return 0;
    }

    int max_delta = 0;
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        int previous = samples[frame_start - SERVO_COUNT + i].pulse_microsec;
        int current = samples[frame_start + i].pulse_microsec;
        int delta = current > previous ? current - previous : previous - current;
        if (delta > max_delta)
        {
            max_delta = delta;
        }
    }
    return max_delta;
}

int main(int argc, char **argv)
{
    std::string trajectory_path;
    int max_delta_microsec = 200;
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--trajectory") == 0 && i + 1 < argc)
        {
            trajectory_path = argv[++i];
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

    if (trajectory_path.empty())
    {
        print_usage(argv[0]);
        return 2;
    }

    std::vector<GaitTrajectorySample> samples;
    std::string error;
    if (!gait_read_trajectory_csv(trajectory_path, &samples, &error) ||
        !gait_validate_trajectory(samples, max_delta_microsec, &error))
    {
        fprintf(stderr, "Invalid trajectory: %s\n", error.c_str());
        return 1;
    }

    printf("Dry-run replay for %s\n", trajectory_path.c_str());
    printf("This tool does not open pigpio, I2C, or command servos.\n");
    for (size_t frame_start = 0; frame_start < samples.size(); frame_start += SERVO_COUNT)
    {
        const GaitTrajectorySample &frame = samples[frame_start];
        printf("time=%04dms phase=%s step=%d max_delta_us=%d\n",
               frame.timestamp_ms,
               frame.phase.c_str(),
               frame.step,
               frame_max_delta(samples, frame_start));
    }

    return 0;
}
