#include "pca9685.h"
#include "semantic_pose.h"
#include "teleop_gait_library.h"
#include "teleop_state.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

static volatile sig_atomic_t g_should_stop = 0;

struct RobotNodeOptions
{
    std::string profile_path;
    std::string poses_dir;
    std::string bind_address;
    int port;
    int i2c_bus;
    int address;
    bool execute;
};

struct MovementLoop
{
    TeleopMovementType movement;
    std::vector<GaitTrajectorySample> samples;
};

static void signal_handler(int)
{
    g_should_stop = 1;
}

static uint64_t now_ms()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

static void print_usage(const char *program)
{
    printf("Usage: %s --profile FILE --poses-dir DIR [--bind A.B.C.D] [--port N] [--execute] [--i2c-bus N] [--address 0x40]\n", program);
    printf("Without --execute this receives commands, checks safety state, and prints dry-run telemetry only.\n");
}

static int parse_int_auto_base(const char *value)
{
    return (int)strtol(value, 0, 0);
}

static bool parse_args(int argc, char **argv, RobotNodeOptions *options)
{
    options->bind_address = "0.0.0.0";
    options->port = 45454;
    options->i2c_bus = PCA9685_DEFAULT_I2C_BUS;
    options->address = PCA9685_DEFAULT_ADDRESS;
    options->execute = false;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--profile") == 0 && i + 1 < argc)
        {
            options->profile_path = argv[++i];
        }
        else if (strcmp(argv[i], "--poses-dir") == 0 && i + 1 < argc)
        {
            options->poses_dir = argv[++i];
        }
        else if (strcmp(argv[i], "--bind") == 0 && i + 1 < argc)
        {
            options->bind_address = argv[++i];
        }
        else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc)
        {
            options->port = atoi(argv[++i]);
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

    return !options->profile_path.empty() &&
           !options->poses_dir.empty() &&
           options->port > 0 &&
           options->port <= 65535;
}

static bool bind_socket(const RobotNodeOptions &options, int *sock)
{
    *sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (*sock < 0)
    {
        perror("socket");
        return false;
    }

    int reuse = 1;
    setsockopt(*sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons((uint16_t)options.port);
    if (inet_pton(AF_INET, options.bind_address.c_str(), &address.sin_addr) != 1)
    {
        fprintf(stderr, "Bind address must be an IPv4 address: %s\n", options.bind_address.c_str());
        close(*sock);
        return false;
    }
    if (bind(*sock, (struct sockaddr *)&address, sizeof(address)) != 0)
    {
        perror("bind");
        close(*sock);
        return false;
    }
    return true;
}

static bool compile_loops(const SemanticRobotProfile &profile,
                          const std::string &poses_dir,
                          std::vector<MovementLoop> *loops,
                          std::string *error)
{
    TeleopGaitTiming timing = teleop_gait_default_timing();
    TeleopMovementType movements[] = {
        TELEOP_MOVEMENT_FORWARD,
        TELEOP_MOVEMENT_BACKWARD,
        TELEOP_MOVEMENT_LEFT,
        TELEOP_MOVEMENT_RIGHT,
        TELEOP_MOVEMENT_ROTATE_LEFT,
        TELEOP_MOVEMENT_ROTATE_RIGHT
    };

    loops->clear();
    for (size_t i = 0; i < sizeof(movements) / sizeof(movements[0]); i++)
    {
        MovementLoop loop;
        loop.movement = movements[i];
        if (!teleop_gait_compile_loop(movements[i], profile, poses_dir, timing, &loop.samples, error))
        {
            return false;
        }
        loops->push_back(loop);
    }
    return true;
}

static const std::vector<GaitTrajectorySample> *find_loop(const std::vector<MovementLoop> &loops,
                                                          TeleopMovementType movement)
{
    for (size_t i = 0; i < loops.size(); i++)
    {
        if (loops[i].movement == movement)
        {
            return &loops[i].samples;
        }
    }
    return 0;
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

static uint64_t scaled_frame_delay_ms(const std::vector<GaitTrajectorySample> &samples,
                                      size_t frame_start,
                                      double speed_scale)
{
    size_t next_frame = frame_start + SERVO_COUNT;
    if (next_frame >= samples.size())
    {
        next_frame = 0;
    }
    int current_ms = samples[frame_start].timestamp_ms;
    int next_ms = samples[next_frame].timestamp_ms;
    if (next_frame == 0)
    {
        next_ms += samples[samples.size() - SERVO_COUNT].timestamp_ms;
    }
    int delta_ms = next_ms - current_ms;
    if (delta_ms <= 0)
    {
        delta_ms = 1;
    }
    uint64_t scaled = (uint64_t)((double)delta_ms / speed_scale);
    return scaled == 0 ? 1 : scaled;
}

static size_t loop_restart_frame(const std::vector<GaitTrajectorySample> &samples)
{
    if (samples.empty())
    {
        return 0;
    }

    std::string first_phase = samples[0].phase;
    size_t restart_frame = 0;
    for (size_t frame_start = 0; frame_start < samples.size(); frame_start += SERVO_COUNT)
    {
        if (samples[frame_start].phase != first_phase)
        {
            break;
        }
        restart_frame = frame_start;
    }
    return restart_frame;
}

static bool frame_to_pose(const std::vector<GaitTrajectorySample> &samples,
                          size_t frame_start,
                          const std::string &name,
                          GaitPose *pose,
                          std::string *error)
{
    if (pose == 0 || frame_start + SERVO_COUNT > samples.size())
    {
        if (error != 0) { *error = "cannot convert incomplete trajectory frame to pose"; }
        return false;
    }

    gait_pose_init(pose);
    pose->name = name;
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        const GaitTrajectorySample &sample = samples[frame_start + i];
        if (!servo_is_valid_index(sample.channel))
        {
            if (error != 0) { *error = "trajectory frame contains invalid servo channel"; }
            return false;
        }
        pose->channel_present[sample.channel] = true;
        pose->pulse_microsec[sample.channel] = sample.pulse_microsec;
    }
    return gait_pose_validate(*pose, error);
}

static bool build_neutral_return(const GaitPose &current_pose,
                                 const GaitPose &neutral_pose,
                                 const TeleopGaitTiming &timing,
                                 std::vector<GaitTrajectorySample> *samples,
                                 std::string *error)
{
    if (samples == 0)
    {
        if (error != 0) { *error = "neutral return samples output is null"; }
        return false;
    }
    samples->clear();
    return gait_append_pose_transition(current_pose,
                                       neutral_pose,
                                       "return_neutral",
                                       timing.phase_duration_ms,
                                       timing.phase_steps * 2,
                                       0,
                                       timing.max_delta_microsec,
                                       samples,
                                       error);
}

int main(int argc, char **argv)
{
    RobotNodeOptions options;
    if (!parse_args(argc, argv, &options))
    {
        print_usage(argv[0]);
        return 2;
    }

    SemanticRobotProfile profile;
    std::string error;
    if (!semantic_profile_load_json(options.profile_path, &profile, &error))
    {
        fprintf(stderr, "Invalid semantic profile: %s\n", error.c_str());
        return 1;
    }

    GaitPose neutral_pose;
    if (!semantic_pose_resolve(profile, options.poses_dir, "neutral_stand", &neutral_pose, &error))
    {
        fprintf(stderr, "Failed to resolve neutral_stand: %s\n", error.c_str());
        return 1;
    }

    std::vector<MovementLoop> loops;
    if (!compile_loops(profile, options.poses_dir, &loops, &error))
    {
        fprintf(stderr, "Failed to compile teleop gait loops: %s\n", error.c_str());
        return 1;
    }

    int sock = -1;
    if (!bind_socket(options, &sock))
    {
        return 1;
    }

    Pca9685Device device;
    if (options.execute)
    {
        device = pca9685_make_device(options.i2c_bus, options.address);
        if (!pca9685_open(&device) || !pca9685_set_pwm_frequency(&device, PCA9685_SERVO_FREQUENCY_HZ))
        {
            fprintf(stderr, "Failed to open PCA9685 on bus %d address 0x%02X.\n", options.i2c_bus, options.address);
            close(sock);
            return 1;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    TeleopState state;
    teleop_state_init(&state, 0);
    TeleopMovementType playback_movement = TELEOP_MOVEMENT_STOP;
    size_t playback_frame = 0;
    uint64_t next_frame_ms = 0;
    uint64_t next_print_ms = 0;
    TeleopGaitTiming timing = teleop_gait_default_timing();
    GaitPose last_commanded_pose = neutral_pose;
    std::vector<GaitTrajectorySample> neutral_return_samples;
    bool neutral_return_active = false;
    size_t neutral_return_frame = 0;
    uint64_t next_neutral_frame_ms = 0;

    printf("Teleop robot node listening on %s:%d\n", options.bind_address.c_str(), options.port);
    printf("Mode: %s\n", options.execute ? "EXECUTE" : "dry run; no hardware will be commanded");

    while (!g_should_stop)
    {
        uint64_t now = now_ms();
        teleop_state_tick(&state, now);

        fd_set read_set;
        FD_ZERO(&read_set);
        FD_SET(sock, &read_set);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        int ready = select(sock + 1, &read_set, 0, 0, &tv);
        if (ready > 0 && FD_ISSET(sock, &read_set))
        {
            char buffer[512];
            struct sockaddr_in sender;
            socklen_t sender_len = sizeof(sender);
            ssize_t received = recvfrom(sock,
                                        buffer,
                                        sizeof(buffer) - 1,
                                        0,
                                        (struct sockaddr *)&sender,
                                        &sender_len);
            if (received > 0)
            {
                buffer[received] = '\0';
                TeleopIntent intent;
                std::string parse_error;
                if (teleop_intent_parse(buffer, &intent, &parse_error))
                {
                    teleop_state_apply_intent(&state, intent, now_ms(), &parse_error);
                }
                else
                {
                    state.rejected_count++;
                    state.safety_state = TELEOP_SAFETY_INVALID;
                }

                TeleopTelemetry telemetry = teleop_state_make_telemetry(state, now_ms(), options.execute);
                std::string telemetry_packet = teleop_telemetry_serialize(telemetry);
                sendto(sock,
                       telemetry_packet.c_str(),
                       telemetry_packet.size(),
                       0,
                       (struct sockaddr *)&sender,
                       sender_len);
            }
        }

        now = now_ms();
        teleop_state_tick(&state, now);
        const std::vector<GaitTrajectorySample> *active_loop = find_loop(loops, state.active_movement);
        if (neutral_return_active)
        {
            if (now >= next_neutral_frame_ms)
            {
                const GaitTrajectorySample &frame = neutral_return_samples[neutral_return_frame];
                if (options.execute && !write_frame(&device, neutral_return_samples, neutral_return_frame))
                {
                    break;
                }
                if (!frame_to_pose(neutral_return_samples,
                                   neutral_return_frame,
                                   "last_commanded",
                                   &last_commanded_pose,
                                   &error))
                {
                    fprintf(stderr, "Failed to track neutral return frame: %s\n", error.c_str());
                    break;
                }
                if (!options.execute && now >= next_print_ms)
                {
                    printf("dry-run active=stop phase=%s step=%d speed=%.2f state=%s safety=%s\n",
                           frame.phase.c_str(),
                           frame.step,
                           state.speed_scale,
                           teleop_run_state_to_string(state.run_state).c_str(),
                           teleop_safety_state_to_string(state.safety_state).c_str());
                    next_print_ms = now + 500;
                }

                uint64_t delay_ms = scaled_frame_delay_ms(neutral_return_samples,
                                                          neutral_return_frame,
                                                          state.speed_scale);
                neutral_return_frame += SERVO_COUNT;
                if (neutral_return_frame >= neutral_return_samples.size())
                {
                    neutral_return_active = false;
                    neutral_return_frame = 0;
                    next_neutral_frame_ms = 0;
                    last_commanded_pose = neutral_pose;
                }
                else
                {
                    next_neutral_frame_ms = now + delay_ms;
                }
            }
        }
        else if (active_loop == 0)
        {
            if (playback_movement != TELEOP_MOVEMENT_STOP)
            {
                if (!build_neutral_return(last_commanded_pose,
                                          neutral_pose,
                                          timing,
                                          &neutral_return_samples,
                                          &error))
                {
                    fprintf(stderr, "Failed to build neutral return: %s\n", error.c_str());
                    break;
                }
                neutral_return_active = true;
                neutral_return_frame = 0;
                next_neutral_frame_ms = now;
            }
            playback_movement = TELEOP_MOVEMENT_STOP;
            playback_frame = 0;
            next_frame_ms = 0;
        }
        else
        {
            if (playback_movement != state.active_movement)
            {
                playback_movement = state.active_movement;
                playback_frame = 0;
                next_frame_ms = now;
            }

            if (now >= next_frame_ms)
            {
                const GaitTrajectorySample &frame = (*active_loop)[playback_frame];
                if (options.execute && !write_frame(&device, *active_loop, playback_frame))
                {
                    break;
                }
                if (!frame_to_pose(*active_loop,
                                   playback_frame,
                                   "last_commanded",
                                   &last_commanded_pose,
                                   &error))
                {
                    fprintf(stderr, "Failed to track active teleop frame: %s\n", error.c_str());
                    break;
                }
                if (!options.execute && now >= next_print_ms)
                {
                    printf("dry-run active=%s phase=%s step=%d speed=%.2f state=%s safety=%s\n",
                           teleop_movement_to_string(state.active_movement).c_str(),
                           frame.phase.c_str(),
                           frame.step,
                           state.speed_scale,
                           teleop_run_state_to_string(state.run_state).c_str(),
                           teleop_safety_state_to_string(state.safety_state).c_str());
                    next_print_ms = now + 500;
                }

                uint64_t delay_ms = scaled_frame_delay_ms(*active_loop, playback_frame, state.speed_scale);
                playback_frame += SERVO_COUNT;
                if (playback_frame >= active_loop->size())
                {
                    playback_frame = loop_restart_frame(*active_loop);
                }
                next_frame_ms = now + delay_ms;
            }
        }
    }

    if (options.execute)
    {
        pca9685_close(&device);
    }
    close(sock);
    return 0;
}
