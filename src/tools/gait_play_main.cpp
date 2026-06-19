#include "gait_trajectory.h"
#include "pca9685.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

struct PlayOptions
{
    std::string trajectory_path;
    int max_delta_microsec;
    int i2c_bus;
    int address;
    double speed;
    bool execute;
};

static void print_usage(const char *program)
{
    printf("Usage: %s --trajectory FILE.csv [--speed SCALE] [--max-delta-us N] [--i2c-bus N] [--address 0x40] [--execute]\n", program);
    printf("Without --execute this is a dry run and does not open pigpio, I2C, or command servos.\n");
}

static int parse_int_auto_base(const char *value)
{
    return (int)strtol(value, 0, 0);
}

static bool parse_args(int argc, char **argv, PlayOptions *options)
{
    options->max_delta_microsec = 80;
    options->i2c_bus = PCA9685_DEFAULT_I2C_BUS;
    options->address = PCA9685_DEFAULT_ADDRESS;
    options->speed = 0.10;
    options->execute = false;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--trajectory") == 0 && i + 1 < argc)
        {
            options->trajectory_path = argv[++i];
        }
        else if (strcmp(argv[i], "--max-delta-us") == 0 && i + 1 < argc)
        {
            options->max_delta_microsec = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--i2c-bus") == 0 && i + 1 < argc)
        {
            options->i2c_bus = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--address") == 0 && i + 1 < argc)
        {
            options->address = parse_int_auto_base(argv[++i]);
        }
        else if (strcmp(argv[i], "--speed") == 0 && i + 1 < argc)
        {
            options->speed = atof(argv[++i]);
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

    if (options->trajectory_path.empty())
    {
        return false;
    }
    if (options->max_delta_microsec <= 0 || options->speed <= 0.0 || options->speed > 1.0)
    {
        return false;
    }
    return true;
}

static int trajectory_duration_ms(const std::vector<GaitTrajectorySample> &samples)
{
    if (samples.empty())
    {
        return 0;
    }
    return samples[samples.size() - SERVO_COUNT].timestamp_ms;
}

static void print_summary(const PlayOptions &options, const std::vector<GaitTrajectorySample> &samples)
{
    printf("Trajectory: %s\n", options.trajectory_path.c_str());
    printf("Frames: %lu\n", (unsigned long)(samples.size() / SERVO_COUNT));
    printf("Duration: %d ms\n", trajectory_duration_ms(samples));
    printf("Playback speed scale: %.2f\n", options.speed);
    printf("Max per-frame delta: %d us\n", options.max_delta_microsec);
    if (!options.execute)
    {
        printf("Mode: dry run; no hardware will be opened or commanded.\n");
    }
    else
    {
        printf("Mode: EXECUTE on I2C bus %d address 0x%02X.\n", options.i2c_bus, options.address);
    }
}

static bool write_frame(const Pca9685Device *device,
                        const std::vector<GaitTrajectorySample> &samples,
                        size_t frame_start)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        const GaitTrajectorySample &sample = samples[frame_start + i];
        if (!pca9685_set_channel_ticks(device, sample.channel, sample.ticks))
        {
            fprintf(stderr, "Failed to write channel %d ticks=%d\n", sample.channel, sample.ticks);
            return false;
        }
    }
    return true;
}

static void sleep_until_next_frame(const std::vector<GaitTrajectorySample> &samples,
                                   size_t frame_start,
                                   double speed)
{
    size_t next_frame = frame_start + SERVO_COUNT;
    if (next_frame >= samples.size())
    {
        return;
    }

    int current_ms = samples[frame_start].timestamp_ms;
    int next_ms = samples[next_frame].timestamp_ms;
    int delta_ms = next_ms - current_ms;
    if (delta_ms <= 0)
    {
        return;
    }

    double scaled_usec = ((double)delta_ms * 1000.0) / speed;
    usleep((useconds_t)scaled_usec);
}

int main(int argc, char **argv)
{
    PlayOptions options;
    if (!parse_args(argc, argv, &options))
    {
        print_usage(argv[0]);
        return 2;
    }

    std::vector<GaitTrajectorySample> samples;
    std::string error;
    if (!gait_read_trajectory_csv(options.trajectory_path, &samples, &error) ||
        !gait_validate_trajectory(samples, options.max_delta_microsec, &error))
    {
        fprintf(stderr, "Invalid trajectory: %s\n", error.c_str());
        return 1;
    }

    print_summary(options, samples);
    if (!options.execute)
    {
        return 0;
    }

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

    bool ok = true;
    for (size_t frame_start = 0; frame_start < samples.size(); frame_start += SERVO_COUNT)
    {
        const GaitTrajectorySample &frame = samples[frame_start];
        printf("execute time=%04dms phase=%s step=%d\n",
               frame.timestamp_ms,
               frame.phase.c_str(),
               frame.step);
        if (!write_frame(&device, samples, frame_start))
        {
            ok = false;
            break;
        }
        sleep_until_next_frame(samples, frame_start, options.speed);
    }

    pca9685_close(&device);
    return ok ? 0 : 1;
}
