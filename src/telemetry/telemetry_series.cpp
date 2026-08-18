#include "telemetry_series.h"

TelemetrySeriesConfig telemetry_series_default_config()
{
    TelemetrySeriesConfig config;
    config.window_ms = 30000;
    config.max_points_per_series = 600;
    return config;
}

TelemetrySeriesStore::TelemetrySeriesStore(const TelemetrySeriesConfig &config)
    : config_(config)
{
}

bool TelemetrySeriesStore::add_sample(const SensorTelemetrySample &sample, std::string *error)
{
    if (!sensor_telemetry_validate(sample, error))
    {
        return false;
    }

    TelemetryPoint point;
    point.timestamp_ms = sample.timestamp_ms;
    point.value = sample.value;
    point.status = sample.status;
    series_[sample.name].push_back(point);
    trim_series(sample.name, sample.timestamp_ms);
    return true;
}

std::vector<TelemetryPoint> TelemetrySeriesStore::points(const std::string &name) const
{
    std::map<std::string, std::vector<TelemetryPoint> >::const_iterator found = series_.find(name);
    if (found == series_.end())
    {
        return std::vector<TelemetryPoint>();
    }
    return found->second;
}

std::vector<std::string> TelemetrySeriesStore::names() const
{
    std::vector<std::string> result;
    for (std::map<std::string, std::vector<TelemetryPoint> >::const_iterator it = series_.begin();
         it != series_.end();
         ++it)
    {
        result.push_back(it->first);
    }
    return result;
}

size_t TelemetrySeriesStore::series_count() const
{
    return series_.size();
}

size_t TelemetrySeriesStore::point_count(const std::string &name) const
{
    std::map<std::string, std::vector<TelemetryPoint> >::const_iterator found = series_.find(name);
    if (found == series_.end())
    {
        return 0;
    }
    return found->second.size();
}

void TelemetrySeriesStore::trim_series(const std::string &name, uint64_t newest_timestamp_ms)
{
    std::vector<TelemetryPoint> &points_ref = series_[name];
    size_t first_keep = 0;
    if (config_.window_ms > 0)
    {
        uint64_t oldest_allowed = newest_timestamp_ms > config_.window_ms
            ? newest_timestamp_ms - config_.window_ms
            : 0;
        while (first_keep < points_ref.size() &&
               points_ref[first_keep].timestamp_ms < oldest_allowed)
        {
            first_keep++;
        }
    }
    if (config_.max_points_per_series > 0 &&
        points_ref.size() - first_keep > config_.max_points_per_series)
    {
        first_keep = points_ref.size() - config_.max_points_per_series;
    }
    if (first_keep > 0)
    {
        points_ref.erase(points_ref.begin(), points_ref.begin() + (long)first_keep);
    }
}
