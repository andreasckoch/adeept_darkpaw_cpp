#include "pca9685.h"
#include "servo_calibration.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

struct ServoTestOptions
{
    int channel;
    int range_percent;
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
    printf("Usage: %s [--channel N] [--range-percent N] [--cycles N] [--step-us N] [--step-delay-ms N] [--settle-ms N] [--i2c-bus N] [--address 0x40] [--execute]\n", program);
    printf("Moves one servo from centre to low, centre, high, and centre. Default channel: 5.\n");
    printf("Without --execute this is a dry run and does not open pigpio, I2C, or command servos.\n");
}

static int parse_int_auto_base(const char *value)
{
    return (int)strtol(value, 0, 0);
}

static bool parse_args(int argc, char **argv, ServoTestOptions *options)
{
    options->channel = 5;
    options->range_percent = 25;
    options->cycles = 1;
    options->step_microsec = 25;
    options->step_delay_ms = 100;
    options->settle_ms = 800;
    options->i2c_bus = PCA9685_DEFAULT_I2C_BUS;
    options->address = PCA9685_DEFAULT_ADDRESS;
    options->execute = false;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--channel") == 0 && i + 1 < argc)
        {
            options->channel = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--range-percent") == 0 && i + 1 < argc)
        {
            options->range_percent = atoi(argv[++i]);
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

    return servo_is_valid_index(options->channel) &&
           options->range_percent > 0 && options->range_percent <= 50 &&
           options->cycles > 0 &&
           options->step_microsec > 0 &&
           options->step_delay_ms > 0 &&
           options->settle_ms >= 0 &&
           options->i2c_bus >= 0;
}

static bool write_pulse(const Pca9685Device *device, int channel, int pulse_microsec)
{
    int ticks = pca9685_pulse_microseconds_to_ticks(pulse_microsec);
    printf("command channel=%d pulse=%dus ticks=%d\n", channel, pulse_microsec, ticks);
    return pca9685_set_channel_ticks(device, channel, ticks);
}

static bool move_to(const Pca9685Device *device,
                    const ServoTestOptions &options,
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

        if (!write_pulse(device, options.channel, next_pulse))
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
    ServoTestOptions options;
    if (!parse_args(argc, argv, &options))
    {
        print_usage(argv[0]);
        return 2;
    }

    ServoPulseLimits limits;
    if (!servo_get_pulse_limits(options.channel, &limits))
    {
        fprintf(stderr, "No calibration limits for channel %d.\n", options.channel);
        return 1;
    }

    int centre = (limits.min_microsec + limits.max_microsec) / 2;
    int excursion = ((limits.max_microsec - limits.min_microsec) * options.range_percent) / 100;
    int low = centre - excursion;
    int high = centre + excursion;

    printf("Servo channel: %d\n", options.channel);
    printf("Calibrated range: %d-%d us\n", limits.min_microsec, limits.max_microsec);
    printf("Test sequence: %d -> %d -> %d -> %d -> %d us\n", centre, low, centre, high, centre);
    printf("Cycles: %d, step: %d us every %d ms, settle: %d ms\n",
           options.cycles, options.step_microsec, options.step_delay_ms, options.settle_ms);

    if (!options.execute)
    {
        printf("Mode: dry run; no hardware will be opened or commanded.\n");
        return 0;
    }

    printf("Mode: EXECUTE on I2C bus %d address 0x%02X. Only channel %d will be written.\n",
           options.i2c_bus, options.address, options.channel);
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

    bool ok = write_pulse(&device, options.channel, centre);
    if (ok && options.settle_ms > 0)
    {
        usleep((useconds_t)options.settle_ms * 1000U);
    }

    int current = centre;
    for (int cycle = 0; ok && cycle < options.cycles; cycle++)
    {
        printf("cycle %d/%d\n", cycle + 1, options.cycles);
        ok = move_to(&device, options, &current, low) &&
             move_to(&device, options, &current, centre) &&
             move_to(&device, options, &current, high) &&
             move_to(&device, options, &current, centre);
    }

    pca9685_close(&device);
    return ok ? 0 : 1;
}
