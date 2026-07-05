#ifndef SENSOR_TELEMETRY_H_
#define SENSOR_TELEMETRY_H_

#include <stdint.h>
#include <string>

enum SensorTelemetryStatus
{
    SENSOR_TELEMETRY_OK = 0,
    SENSOR_TELEMETRY_UNAVAILABLE,
    SENSOR_TELEMETRY_STALE,
    SENSOR_TELEMETRY_ERROR
};

struct SensorTelemetrySample
{
    uint32_t sequence_id;
    uint64_t timestamp_ms;
    std::string name;
    double value;
    std::string unit;
    SensorTelemetryStatus status;
};

void sensor_telemetry_init(SensorTelemetrySample *sample);
std::string sensor_telemetry_status_to_string(SensorTelemetryStatus status);
bool sensor_telemetry_status_from_string(const std::string &text, SensorTelemetryStatus *status);
bool sensor_telemetry_name_is_supported(const std::string &name);
bool sensor_telemetry_validate(const SensorTelemetrySample &sample, std::string *error);
std::string sensor_telemetry_serialize(const SensorTelemetrySample &sample);
bool sensor_telemetry_parse(const std::string &packet, SensorTelemetrySample *sample, std::string *error);

#endif /* SENSOR_TELEMETRY_H_ */
