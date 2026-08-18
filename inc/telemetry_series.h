#ifndef TELEMETRY_SERIES_H_
#define TELEMETRY_SERIES_H_

#include "sensor_telemetry.h"

#include <map>
#include <string>
#include <vector>

struct TelemetryPoint
{
    uint64_t timestamp_ms;
    double value;
    SensorTelemetryStatus status;
};

struct TelemetrySeriesConfig
{
    uint64_t window_ms;
    size_t max_points_per_series;
};

class TelemetrySeriesStore
{
public:
    explicit TelemetrySeriesStore(const TelemetrySeriesConfig &config);

    bool add_sample(const SensorTelemetrySample &sample, std::string *error);
    std::vector<TelemetryPoint> points(const std::string &name) const;
    std::vector<std::string> names() const;
    size_t series_count() const;
    size_t point_count(const std::string &name) const;

private:
    void trim_series(const std::string &name, uint64_t newest_timestamp_ms);

    TelemetrySeriesConfig config_;
    std::map<std::string, std::vector<TelemetryPoint> > series_;
};

TelemetrySeriesConfig telemetry_series_default_config();

#endif /* TELEMETRY_SERIES_H_ */
