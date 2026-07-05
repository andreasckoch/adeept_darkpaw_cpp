#include "sensor_telemetry.h"

#include <math.h>
#include <sstream>
#include <stdlib.h>

static const char *SENSOR_TELEMETRY_MAGIC = "SPIDER_SENSOR_V1";

static bool is_safe_token(const std::string &text)
{
    if (text.empty())
    {
        return false;
    }
    for (size_t i = 0; i < text.size(); i++)
    {
        char c = text[i];
        if (!((c >= 'a' && c <= 'z') ||
              (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') ||
              c == '_' || c == '.' || c == '-' || c == '/'))
        {
            return false;
        }
    }
    return true;
}

void sensor_telemetry_init(SensorTelemetrySample *sample)
{
    sample->sequence_id = 0;
    sample->timestamp_ms = 0;
    sample->name.clear();
    sample->value = 0.0;
    sample->unit.clear();
    sample->status = SENSOR_TELEMETRY_UNAVAILABLE;
}

std::string sensor_telemetry_status_to_string(SensorTelemetryStatus status)
{
    switch (status)
    {
        case SENSOR_TELEMETRY_OK:
            return "ok";
        case SENSOR_TELEMETRY_UNAVAILABLE:
            return "unavailable";
        case SENSOR_TELEMETRY_STALE:
            return "stale";
        case SENSOR_TELEMETRY_ERROR:
            return "error";
    }
    return "error";
}

bool sensor_telemetry_status_from_string(const std::string &text, SensorTelemetryStatus *status)
{
    if (status == 0)
    {
        return false;
    }
    if (text == "ok")
    {
        *status = SENSOR_TELEMETRY_OK;
    }
    else if (text == "unavailable")
    {
        *status = SENSOR_TELEMETRY_UNAVAILABLE;
    }
    else if (text == "stale")
    {
        *status = SENSOR_TELEMETRY_STALE;
    }
    else if (text == "error")
    {
        *status = SENSOR_TELEMETRY_ERROR;
    }
    else
    {
        return false;
    }
    return true;
}

bool sensor_telemetry_name_is_supported(const std::string &name)
{
    static const char *SUPPORTED_PREFIXES[] = {
        "heartbeat",
        "camera.",
        "runtime.",
        "teleop.",
        "network.",
        "system.",
        "imu.",
        "battery.",
        "range.",
        "servo."
    };

    for (size_t i = 0; i < sizeof(SUPPORTED_PREFIXES) / sizeof(SUPPORTED_PREFIXES[0]); i++)
    {
        std::string prefix = SUPPORTED_PREFIXES[i];
        if (name == prefix || name.find(prefix) == 0)
        {
            return true;
        }
    }
    return false;
}

bool sensor_telemetry_validate(const SensorTelemetrySample &sample, std::string *error)
{
    if (sample.timestamp_ms == 0)
    {
        if (error != 0) { *error = "sensor telemetry timestamp_ms must be non-zero"; }
        return false;
    }
    if (!is_safe_token(sample.name) || !sensor_telemetry_name_is_supported(sample.name))
    {
        if (error != 0) { *error = "sensor telemetry name is invalid or unsupported"; }
        return false;
    }
    if (!sample.unit.empty() && !is_safe_token(sample.unit))
    {
        if (error != 0) { *error = "sensor telemetry unit is invalid"; }
        return false;
    }
    if (sample.status == SENSOR_TELEMETRY_OK && !isfinite(sample.value))
    {
        if (error != 0) { *error = "ok sensor telemetry value must be finite"; }
        return false;
    }
    return true;
}

std::string sensor_telemetry_serialize(const SensorTelemetrySample &sample)
{
    std::ostringstream stream;
    stream << SENSOR_TELEMETRY_MAGIC << " "
           << sample.sequence_id << " "
           << sample.timestamp_ms << " "
           << sample.name << " "
           << sample.value << " "
           << (sample.unit.empty() ? "-" : sample.unit) << " "
           << sensor_telemetry_status_to_string(sample.status);
    return stream.str();
}

bool sensor_telemetry_parse(const std::string &packet, SensorTelemetrySample *sample, std::string *error)
{
    if (sample == 0)
    {
        if (error != 0) { *error = "sensor telemetry output is null"; }
        return false;
    }

    std::istringstream stream(packet);
    std::string magic;
    std::string unit;
    std::string status_text;
    SensorTelemetrySample parsed;
    sensor_telemetry_init(&parsed);

    stream >> magic
           >> parsed.sequence_id
           >> parsed.timestamp_ms
           >> parsed.name
           >> parsed.value
           >> unit
           >> status_text;
    if (!stream || magic != SENSOR_TELEMETRY_MAGIC)
    {
        if (error != 0) { *error = "packet is not a valid SPIDER_SENSOR_V1 message"; }
        return false;
    }
    parsed.unit = unit == "-" ? "" : unit;
    if (!sensor_telemetry_status_from_string(status_text, &parsed.status))
    {
        if (error != 0) { *error = "sensor telemetry status is invalid"; }
        return false;
    }
    if (!sensor_telemetry_validate(parsed, error))
    {
        return false;
    }

    *sample = parsed;
    return true;
}
