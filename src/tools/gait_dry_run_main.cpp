#include "gait_controller.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void print_usage(const char *program)
{
    printf("Usage: %s [--vx MPS] [--vy MPS] [--yaw RADPS]\n", program);
}

static bool parse_float(const char *value, float *parsed)
{
    char *end = 0;
    float result = strtof(value, &end);
    if (end == value || *end != '\0')
    {
        return false;
    }

    *parsed = result;
    return true;
}

static bool parse_args(int argc, char **argv, MotionCommand *command)
{
    *command = motion_command_zero();

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--vx") == 0)
        {
            if (i + 1 >= argc || !parse_float(argv[i + 1], &command->velocity_x_mps))
            {
                fprintf(stderr, "Invalid --vx value\n");
                return false;
            }
            i++;
        }
        else if (strcmp(argv[i], "--vy") == 0)
        {
            if (i + 1 >= argc || !parse_float(argv[i + 1], &command->velocity_y_mps))
            {
                fprintf(stderr, "Invalid --vy value\n");
                return false;
            }
            i++;
        }
        else if (strcmp(argv[i], "--yaw") == 0)
        {
            if (i + 1 >= argc || !parse_float(argv[i + 1], &command->yaw_rate_radps))
            {
                fprintf(stderr, "Invalid --yaw value\n");
                return false;
            }
            i++;
        }
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0)
        {
            print_usage(argv[0]);
            exit(0);
        }
        else
        {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            return false;
        }
    }

    return true;
}

static void print_command(const char *label, const MotionCommand *command)
{
    printf("%s: vx=%.3f m/s, vy=%.3f m/s, yaw=%.3f rad/s\n",
           label,
           command->velocity_x_mps,
           command->velocity_y_mps,
           command->yaw_rate_radps);
}

int main(int argc, char **argv)
{
    MotionCommand command;
    if (!parse_args(argc, argv, &command))
    {
        print_usage(argv[0]);
        return 2;
    }

    GaitDryRunResult result;
    if (!gait_controller_dry_run(&command, &result))
    {
        fprintf(stderr, "Failed to generate dry-run gait result\n");
        return 1;
    }

    printf("Adeept Darkpaw gait dry run\n");
    printf("This tool does not open pigpio, I2C, or command servos.\n\n");

    print_command("Raw command", &result.raw_command);
    print_command("Bounded command", &result.bounded_command);
    printf("Raw command valid: %s\n", result.raw_command_valid ? "yes" : "no");
    printf("Selected gait mode: %s\n\n", gait_mode_name(result.mode));

    printf("Servo targets:\n");
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        printf("  servo %2d: %4d us -> %3d ticks%s\n",
               result.targets[i].channel,
               result.targets[i].pulse_microsec,
               result.targets[i].ticks,
               result.targets[i].within_limits ? "" : " (clamped)");
    }

    return result.raw_command_valid ? 0 : 1;
}
