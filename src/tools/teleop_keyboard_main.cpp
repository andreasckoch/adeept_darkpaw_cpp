#include "teleop_message.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

struct KeyboardOptions
{
    std::string host;
    int port;
    int rate_hz;
    int hold_ms;
    double speed_scale;
    bool dry_run;
};

static struct termios g_original_termios;
static bool g_termios_saved = false;

static uint64_t now_ms()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

static void restore_terminal()
{
    if (g_termios_saved)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_original_termios);
    }
}

static bool enable_raw_terminal()
{
    if (tcgetattr(STDIN_FILENO, &g_original_termios) != 0)
    {
        return false;
    }
    g_termios_saved = true;
    atexit(restore_terminal);

    struct termios raw = g_original_termios;
    raw.c_lflag &= (tcflag_t) ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    return tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
}

static void print_usage(const char *program)
{
    printf("Usage: %s [--host A.B.C.D] [--port N] [--rate-hz N] [--hold-ms N] [--speed SCALE] [--dry-run]\n", program);
    printf("Keys: W/A/S/D move, Q/E rotate, space stops, X sends estop, Ctrl-C exits.\n");
    printf("Terminal input has no true key-release events, so no key for --hold-ms streams stop.\n");
}

static bool parse_args(int argc, char **argv, KeyboardOptions *options)
{
    options->host = "127.0.0.1";
    options->port = 45454;
    options->rate_hz = 30;
    options->hold_ms = 180;
    options->speed_scale = 0.50;
    options->dry_run = false;

    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "--host") == 0 && i + 1 < argc)
        {
            options->host = argv[++i];
        }
        else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc)
        {
            options->port = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--rate-hz") == 0 && i + 1 < argc)
        {
            options->rate_hz = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--hold-ms") == 0 && i + 1 < argc)
        {
            options->hold_ms = atoi(argv[++i]);
        }
        else if (strcmp(argv[i], "--speed") == 0 && i + 1 < argc)
        {
            options->speed_scale = atof(argv[++i]);
        }
        else if (strcmp(argv[i], "--dry-run") == 0)
        {
            options->dry_run = true;
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

    return options->port > 0 &&
           options->port <= 65535 &&
           options->rate_hz > 0 &&
           options->rate_hz <= 100 &&
           options->hold_ms >= 0 &&
           options->speed_scale >= 0.0 &&
           options->speed_scale <= 1.0;
}

static bool make_destination(const KeyboardOptions &options, struct sockaddr_in *address)
{
    memset(address, 0, sizeof(*address));
    address->sin_family = AF_INET;
    address->sin_port = htons((uint16_t)options.port);
    return inet_pton(AF_INET, options.host.c_str(), &address->sin_addr) == 1;
}

static bool read_key(char *key, int timeout_ms)
{
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(STDIN_FILENO, &read_set);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ready = select(STDIN_FILENO + 1, &read_set, 0, 0, &tv);
    if (ready <= 0)
    {
        return false;
    }

    return read(STDIN_FILENO, key, 1) == 1;
}

int main(int argc, char **argv)
{
    KeyboardOptions options;
    if (!parse_args(argc, argv, &options))
    {
        print_usage(argv[0]);
        return 2;
    }

    struct sockaddr_in destination;
    if (!make_destination(options, &destination))
    {
        fprintf(stderr, "Host must be an IPv4 address for this first teleop client: %s\n", options.host.c_str());
        return 2;
    }

    int sock = -1;
    if (!options.dry_run)
    {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0)
        {
            perror("socket");
            return 1;
        }
    }

    if (!enable_raw_terminal())
    {
        fprintf(stderr, "Failed to enable raw terminal input.\n");
        if (sock >= 0) { close(sock); }
        return 1;
    }

    printf("Streaming teleop intent to %s:%d%s\n",
           options.host.c_str(),
           options.port,
           options.dry_run ? " (dry run)" : "");
    printf("W/A/S/D move, Q/E rotate, space stops, X estops, Ctrl-C exits.\n");

    uint32_t sequence_id = 1;
    TeleopMovementType current_movement = TELEOP_MOVEMENT_STOP;
    bool current_estop = false;
    bool current_enabled = false;
    uint64_t last_key_ms = 0;
    int period_ms = 1000 / options.rate_hz;
    if (period_ms <= 0)
    {
        period_ms = 1;
    }

    while (true)
    {
        char key = 0;
        if (read_key(&key, period_ms))
        {
            bool key_estop = false;
            bool key_enabled = false;
            TeleopMovementType key_movement = TELEOP_MOVEMENT_STOP;
            if (teleop_movement_from_key(key, &key_movement, &key_estop, &key_enabled))
            {
                current_movement = key_movement;
                current_estop = key_estop;
                current_enabled = key_enabled;
                last_key_ms = now_ms();
            }
        }

        uint64_t timestamp_ms = now_ms();
        if (last_key_ms == 0 || timestamp_ms - last_key_ms > (uint64_t)options.hold_ms)
        {
            current_movement = TELEOP_MOVEMENT_STOP;
            current_enabled = false;
            current_estop = false;
        }

        TeleopIntent intent;
        teleop_intent_init(&intent);
        intent.sequence_id = sequence_id++;
        intent.timestamp_ms = timestamp_ms;
        intent.movement = current_movement;
        intent.speed_scale = options.speed_scale;
        intent.enabled = current_enabled;
        intent.estop = current_estop;

        std::string packet = teleop_intent_serialize(intent);
        if (options.dry_run)
        {
            printf("%s\n", packet.c_str());
        }
        else
        {
            ssize_t sent = sendto(sock,
                                  packet.c_str(),
                                  packet.size(),
                                  0,
                                  (struct sockaddr *)&destination,
                                  sizeof(destination));
            if (sent < 0)
            {
                perror("sendto");
                restore_terminal();
                close(sock);
                return 1;
            }
        }
    }

    restore_terminal();
    if (sock >= 0) { close(sock); }
    return 0;
}
