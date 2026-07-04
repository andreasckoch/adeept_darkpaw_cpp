#include "pca9685.h"
#include "semantic_profile.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

struct SemanticJointTestOptions
{
    std::string profile_path;
    std::string leg_name;
    std::string axis;
    int cycles;
    int step_microsec;
    int step_delay_ms;
    int settle_ms;
    int i2c_bus;
    int address;
    bool execute;
};

static void print_usage(const char *program)
{
    printf("Usage: %s --profile FILE.json --leg NAME --axis fore_aft|lift|stance [--cycles N] [--step-us N] [--step-delay-ms N] [--settle-ms N] [--i2c-bus N] [--address 0x40] [--execute]\n", program);
    printf("Tests profile values as neutral -> first endpoint -> neutral -> second endpoint -> neutral.\n");
    printf("Without --execute this is a dry run and does not open pigpio, I2C, or command servos.\n");
}

static int parse_int_auto_base(const char *value)
{
    return (int)strtol(value, 0, 0);
}

static bool parse_args(int argc, char **argv, SemanticJointTestOptions *options)
{
    options->cycles = 1;
    options->step_microsec = 25;
    options->step_delay_ms = 100;
    options->settle_ms = 800;
    options->i2c_bus = PCA9685_DEFAULT_I2C_BUS;
    options->address = PCA9685_DEFAULT_ADDRESS;
    options->execute = false;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--profile") == 0 && i + 1 < argc)
        {
            options->profile_path = argv[++i];
        }
        else if (strcmp(argv[i], "--leg") == 0 && i + 1 < argc)
        {
            options->leg_name = argv[++i];
        }
        else if (strcmp(argv[i], "--axis") == 0 && i + 1 < argc)
        {
            options->axis = argv[++i];
        }
        else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc)
        {
            options->cycles = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--step-us") == 0 && i + 1 < argc)
        {
            options->step_microsec = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--step-delay-ms") == 0 && i + 1 < argc)
        {
            options->step_delay_ms = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--settle-ms") == 0 && i + 1 < argc)
        {
            options->settle_ms = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--i2c-bus") == 0 && i + 1 < argc)
        {
            options->i2c_bus = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--address") == 0 && i + 1 < argc)
        {
            options->address = parse_int_auto_base(argv[++i]);
        }
        else if (strcmp(argv[i], "--execute") == 0)
        {
            options->execute = true;
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

    return !options->profile_path.empty() && !options->leg_name.empty() &&
           (options->axis == "fore_aft" || options->axis == "lift" || options->axis == "stance") &&
           options->cycles > 0 && options->step_microsec > 0 &&
           options->step_delay_ms > 0 && options->settle_ms >= 0 &&
           options->i2c_bus >= 0 && options->address > 0 && options->address <= 0x7F;
}

static bool axis_position_names(const std::string &axis, const char **first, const char **neutral, const char **second)
{
    if (axis == "fore_aft")
    {
        *first = "back";
        *neutral = "neutral";
        *second = "front";
        return true;
    }
    if (axis == "lift")
    {
        *first = "down";
        *neutral = "neutral";
        *second = "up";
        return true;
    }
    if (axis == "stance")
    {
        *first = "close";
        *neutral = "neutral";
        *second = "wide";
        return true;
    }
    return false;
}

static bool write_pulse(const Pca9685Device *device, int channel, int pulse_microsec)
{
    int ticks = pca9685_pulse_microseconds_to_ticks(pulse_microsec);
    printf("command channel=%d pulse=%dus ticks=%d\n", channel, pulse_microsec, ticks);
    return pca9685_set_channel_ticks(device, channel, ticks);
}

static bool move_to(const Pca9685Device *device,
                    const SemanticJointTestOptions &options,
                    int channel,
                    int *current_pulse_microsec,
                    int target_pulse_microsec)
{
    int direction = target_pulse_microsec >= *current_pulse_microsec ? 1 : -1;
    int next_pulse = *current_pulse_microsec;
    while (next_pulse != target_pulse_microsec)
    {
        next_pulse += direction * options.step_microsec;
        if ((direction > 0 && next_pulse > target_pulse_microsec) ||
            (direction < 0 && next_pulse < target_pulse_microsec))
        {
            next_pulse = target_pulse_microsec;
        }
        if (!write_pulse(device, channel, next_pulse))
        {
            return false;
        }
        usleep((useconds_t)options.step_delay_ms * 1000U);
    }

    *current_pulse_microsec = target_pulse_microsec;
    if (options.settle_ms > 0)
    {
        usleep((useconds_t)options.settle_ms * 1000U);
    }
    return true;
}

int main(int argc, char **argv)
{
    SemanticJointTestOptions options;
    if (!parse_args(argc, argv, &options))
    {
        print_usage(argv[0]);
        return 2;
    }

    SemanticRobotProfile profile;
    std::string error;
    if (!semantic_profile_load_json(options.profile_path, &profile, &error))
    {
        fprintf(stderr, "Failed to load semantic profile: %s\n", error.c_str());
        return 1;
    }

    const char *first_name = 0;
    const char *neutral_name = 0;
    const char *second_name = 0;
    if (!axis_position_names(options.axis, &first_name, &neutral_name, &second_name))
    {
        fprintf(stderr, "Unsupported semantic axis.\n");
        return 1;
    }

    int channel = -1;
    int first_pulse = 0;
    int neutral_pulse = 0;
    int second_pulse = 0;
    if (!semantic_profile_get_pulse(profile, options.leg_name, options.axis, first_name, &channel, &first_pulse, &error) ||
        !semantic_profile_get_pulse(profile, options.leg_name, options.axis, neutral_name, 0, &neutral_pulse, &error) ||
        !semantic_profile_get_pulse(profile, options.leg_name, options.axis, second_name, 0, &second_pulse, &error))
    {
        fprintf(stderr, "Failed to resolve semantic joint: %s\n", error.c_str());
        return 1;
    }

    printf("Semantic joint: %s.%s (channel %d)\n", options.leg_name.c_str(), options.axis.c_str(), channel);
    printf("Profile values: %s=%d us, %s=%d us, %s=%d us\n",
           first_name, first_pulse, neutral_name, neutral_pulse, second_name, second_pulse);
    printf("Test sequence: %s -> %s -> %s -> %s -> %s\n",
           neutral_name, first_name, neutral_name, second_name, neutral_name);
    printf("Cycles: %d, step: %d us every %d ms, settle: %d ms\n",
           options.cycles, options.step_microsec, options.step_delay_ms, options.settle_ms);

    if (!options.execute)
    {
        printf("Mode: dry run; no hardware will be opened or commanded.\n");
        return 0;
    }

    printf("Mode: EXECUTE on I2C bus %d address 0x%02X. Only channel %d will be written.\n",
           options.i2c_bus, options.address, channel);
    Pca9685Device device = pca9685_make_device(options.i2c_bus, options.address);
    if (!pca9685_open(&device))
    {
        fprintf(stderr, "Failed to open PCA9685.\n");
        return 1;
    }
    if (!pca9685_set_pwm_frequency(&device, PCA9685_SERVO_FREQUENCY_HZ))
    {
        fprintf(stderr, "Failed to set PCA9685 PWM frequency.\n");
        pca9685_close(&device);
        return 1;
    }

    bool ok = write_pulse(&device, channel, neutral_pulse);
    if (ok && options.settle_ms > 0)
    {
        usleep((useconds_t)options.settle_ms * 1000U);
    }

    int current_pulse = neutral_pulse;
    for (int cycle = 0; ok && cycle < options.cycles; cycle++)
    {
        printf("cycle %d/%d\n", cycle + 1, options.cycles);
        ok = move_to(&device, options, channel, &current_pulse, first_pulse) &&
             move_to(&device, options, channel, &current_pulse, neutral_pulse) &&
             move_to(&device, options, channel, &current_pulse, second_pulse) &&
             move_to(&device, options, channel, &current_pulse, neutral_pulse);
    }

    pca9685_close(&device);
    return ok ? 0 : 1;
}
