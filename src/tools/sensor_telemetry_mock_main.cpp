#include "sensor_telemetry.h"

#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>

struct MockOptions
{
    std::string host;
    int port;
    int rate_hz;
};

static uint64_t now_ms()
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)(tv.tv_usec / 1000);
}

static void print_usage(const char *program)
{
    printf("Usage: %s [--host A.B.C.D] [--port N] [--rate-hz N]\n", program);
}

static bool parse_args(int argc, char **argv, MockOptions *options)
{
    options->host = "127.0.0.1";
    options->port = 45455;
    options->rate_hz = 20;
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
    return options->port > 0 && options->port <= 65535 && options->rate_hz > 0 && options->rate_hz <= 100;
}

static bool make_destination(const MockOptions &options, struct sockaddr_in *address)
{
    memset(address, 0, sizeof(*address));
    address->sin_family = AF_INET;
    address->sin_port = htons((uint16_t)options.port);
    return inet_pton(AF_INET, options.host.c_str(), &address->sin_addr) == 1;
}

static bool send_sample(int sock,
                        const struct sockaddr_in &destination,
                        const SensorTelemetrySample &sample)
{
    std::string packet = sensor_telemetry_serialize(sample);
    ssize_t sent = sendto(sock,
                          packet.c_str(),
                          packet.size(),
                          0,
                          (const struct sockaddr *)&destination,
                          sizeof(destination));
    return sent == (ssize_t)packet.size();
}

static SensorTelemetrySample make_sample(uint32_t sequence_id,
                                         uint64_t timestamp_ms,
                                         const char *name,
                                         double value,
                                         const char *unit,
                                         SensorTelemetryStatus status)
{
    SensorTelemetrySample sample;
    sensor_telemetry_init(&sample);
    sample.sequence_id = sequence_id;
    sample.timestamp_ms = timestamp_ms;
    sample.name = name;
    sample.value = value;
    sample.unit = unit;
    sample.status = status;
    return sample;
}

int main(int argc, char **argv)
{
    MockOptions options;
    if (!parse_args(argc, argv, &options))
    {
        print_usage(argv[0]);
        return 2;
    }

    struct sockaddr_in destination;
    if (!make_destination(options, &destination))
    {
        fprintf(stderr, "Host must be an IPv4 address: %s\n", options.host.c_str());
        return 2;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        perror("socket");
        return 1;
    }

    printf("Streaming mock sensor telemetry to %s:%d\n", options.host.c_str(), options.port);
    uint32_t sequence_id = 1;
    int period_us = 1000000 / options.rate_hz;
    while (true)
    {
        uint64_t timestamp = now_ms();
        double t = (double)timestamp / 1000.0;
        std::vector<SensorTelemetrySample> samples;
        samples.push_back(make_sample(sequence_id++, timestamp, "heartbeat", 1.0, "bool", SENSOR_TELEMETRY_OK));
        samples.push_back(make_sample(sequence_id++, timestamp, "camera.connected", 1.0, "bool", SENSOR_TELEMETRY_OK));
        samples.push_back(make_sample(sequence_id++, timestamp, "camera.fps", 24.0 + 2.0 * sin(t), "fps", SENSOR_TELEMETRY_OK));
        samples.push_back(make_sample(sequence_id++, timestamp, "system.cpu_temp_c", 51.0 + 4.0 * sin(t * 0.2), "C", SENSOR_TELEMETRY_OK));
        samples.push_back(make_sample(sequence_id++, timestamp, "network.packet_loss", 0.01 + 0.01 * sin(t * 0.7), "ratio", SENSOR_TELEMETRY_OK));
        samples.push_back(make_sample(sequence_id++, timestamp, "runtime.loop_hz", 30.0 + sin(t * 0.5), "Hz", SENSOR_TELEMETRY_OK));
        samples.push_back(make_sample(sequence_id++, timestamp, "imu.accel_x", 0.0, "mps2", SENSOR_TELEMETRY_UNAVAILABLE));
        samples.push_back(make_sample(sequence_id++, timestamp, "battery.voltage", 0.0, "V", SENSOR_TELEMETRY_UNAVAILABLE));
        for (size_t i = 0; i < samples.size(); i++)
        {
            send_sample(sock, destination, samples[i]);
        }
        usleep((useconds_t)period_us);
    }

    close(sock);
    return 0;
}
