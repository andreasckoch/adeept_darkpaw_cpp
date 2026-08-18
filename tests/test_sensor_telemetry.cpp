#include "sensor_telemetry.h"
#include "telemetry_series.h"

#include <assert.h>

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

static void test_packet_roundtrip()
{
    SensorTelemetrySample sample = make_sample(1, 1000, "system.cpu_temp_c", 52.25, "C", SENSOR_TELEMETRY_OK);
    std::string packet = sensor_telemetry_serialize(sample);
    SensorTelemetrySample parsed;
    std::string error;
    assert(sensor_telemetry_parse(packet, &parsed, &error));
    assert(parsed.sequence_id == sample.sequence_id);
    assert(parsed.timestamp_ms == sample.timestamp_ms);
    assert(parsed.name == sample.name);
    assert(parsed.value == sample.value);
    assert(parsed.unit == sample.unit);
    assert(parsed.status == sample.status);
}

static void test_optional_unavailable_sensor_is_valid()
{
    SensorTelemetrySample sample = make_sample(2, 1200, "imu.accel_x", 0.0, "mps2", SENSOR_TELEMETRY_UNAVAILABLE);
    std::string error;
    assert(sensor_telemetry_validate(sample, &error));
}

static void test_unknown_sensor_is_rejected()
{
    SensorTelemetrySample sample = make_sample(3, 1200, "lidar.secret", 0.0, "m", SENSOR_TELEMETRY_OK);
    std::string error;
    assert(!sensor_telemetry_validate(sample, &error));
}

static void test_series_trimming()
{
    TelemetrySeriesConfig config;
    config.window_ms = 100;
    config.max_points_per_series = 3;
    TelemetrySeriesStore store(config);
    std::string error;
    assert(store.add_sample(make_sample(1, 1000, "runtime.loop_hz", 30.0, "Hz", SENSOR_TELEMETRY_OK), &error));
    assert(store.add_sample(make_sample(2, 1050, "runtime.loop_hz", 31.0, "Hz", SENSOR_TELEMETRY_OK), &error));
    assert(store.add_sample(make_sample(3, 1100, "runtime.loop_hz", 32.0, "Hz", SENSOR_TELEMETRY_OK), &error));
    assert(store.add_sample(make_sample(4, 1150, "runtime.loop_hz", 33.0, "Hz", SENSOR_TELEMETRY_OK), &error));
    assert(store.point_count("runtime.loop_hz") == 3);
    std::vector<TelemetryPoint> points = store.points("runtime.loop_hz");
    assert(points[0].timestamp_ms == 1050);
    assert(points[2].value == 33.0);
}

int main()
{
    test_packet_roundtrip();
    test_optional_unavailable_sensor_is_valid();
    test_unknown_sensor_is_rejected();
    test_series_trimming();
    return 0;
}
