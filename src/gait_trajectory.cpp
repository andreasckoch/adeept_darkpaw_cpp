#include "gait_trajectory.h"

#include "pca9685.h"

#include <fstream>
#include <stdlib.h>
#include <sstream>

static std::string join_path(const std::string &dir, const std::string &file)
{
    if (dir.empty() || dir[dir.size() - 1] == '/')
    {
        return dir + file;
    }
    return dir + "/" + file;
}

static int interpolate_int(int from, int to, int step, int steps)
{
    int delta = to - from;
    if (delta >= 0)
    {
        return from + ((delta * step) + (steps / 2)) / steps;
    }
    return from + ((delta * step) - (steps / 2)) / steps;
}

static bool load_pose_by_name(const std::string &poses_dir, const std::string &pose_name, GaitPose *pose, std::string *error)
{
    return gait_pose_load_json(join_path(poses_dir, pose_name + ".json"), pose, error);
}

static bool validate_phase_delta(const GaitPose &from_pose,
                                 const GaitPose &to_pose,
                                 const GaitPhaseDefinition &phase,
                                 int max_delta_microsec,
                                 std::string *error)
{
    if (max_delta_microsec <= 0)
    {
        return true;
    }

    for (int channel = 0; channel < SERVO_COUNT; channel++)
    {
        int last_pulse = from_pose.pulse_microsec[channel];
        for (int step = 1; step <= phase.steps; step++)
        {
            int pulse = interpolate_int(from_pose.pulse_microsec[channel],
                                        to_pose.pulse_microsec[channel],
                                        step,
                                        phase.steps);
            int delta = pulse > last_pulse ? pulse - last_pulse : last_pulse - pulse;
            if (delta > max_delta_microsec)
            {
                std::stringstream message;
                message << "phase '" << phase.name << "' channel " << channel
                        << " changes by " << delta << " us in one step; limit is "
                        << max_delta_microsec << " us";
                if (error != 0) { *error = message.str(); }
                return false;
            }
            last_pulse = pulse;
        }
    }

    return true;
}

bool gait_compile_trajectory(const GaitDefinition &definition,
                             const std::string &poses_dir,
                             int max_delta_microsec,
                             std::vector<GaitTrajectorySample> *samples,
                             std::string *error)
{
    if (samples == 0)
    {
        if (error != 0) { *error = "samples output is null"; }
        return false;
    }
    samples->clear();

    if (!gait_definition_validate(definition, error))
    {
        return false;
    }

    int phase_start_ms = 0;
    for (size_t phase_index = 0; phase_index < definition.phases.size(); phase_index++)
    {
        const GaitPhaseDefinition &phase = definition.phases[phase_index];
        GaitPose from_pose;
        GaitPose to_pose;

        if (!load_pose_by_name(poses_dir, phase.from_pose, &from_pose, error))
        {
            return false;
        }
        if (!load_pose_by_name(poses_dir, phase.to_pose, &to_pose, error))
        {
            return false;
        }
        if (!validate_phase_delta(from_pose, to_pose, phase, max_delta_microsec, error))
        {
            return false;
        }

        for (int step = 0; step <= phase.steps; step++)
        {
            int timestamp_ms = phase_start_ms + ((phase.duration_ms * step) / phase.steps);
            for (int channel = 0; channel < SERVO_COUNT; channel++)
            {
                int pulse = interpolate_int(from_pose.pulse_microsec[channel],
                                            to_pose.pulse_microsec[channel],
                                            step,
                                            phase.steps);
                int clamped = servo_clamp_pulse_microseconds(channel, pulse);
                if (clamped != pulse)
                {
                    std::stringstream message;
                    message << "compiled phase '" << phase.name << "' channel " << channel
                            << " generated out-of-range pulse " << pulse;
                    if (error != 0) { *error = message.str(); }
                    return false;
                }

                GaitTrajectorySample sample;
                sample.timestamp_ms = timestamp_ms;
                sample.phase = phase.name;
                sample.step = step;
                sample.channel = channel;
                sample.pulse_microsec = pulse;
                sample.ticks = pca9685_pulse_microseconds_to_ticks(pulse);
                samples->push_back(sample);
            }
        }

        phase_start_ms += phase.duration_ms;
    }

    return gait_validate_trajectory(*samples, max_delta_microsec, error);
}

bool gait_write_trajectory_csv(const std::string &path,
                               const std::vector<GaitTrajectorySample> &samples,
                               std::string *error)
{
    std::ofstream output(path.c_str());
    if (!output)
    {
        if (error != 0) { *error = "failed to open output CSV: " + path; }
        return false;
    }

    output << "timestamp_ms,phase,step,channel,pulse_microsec,ticks\n";
    for (size_t i = 0; i < samples.size(); i++)
    {
        output << samples[i].timestamp_ms << ","
               << samples[i].phase << ","
               << samples[i].step << ","
               << samples[i].channel << ","
               << samples[i].pulse_microsec << ","
               << samples[i].ticks << "\n";
    }

    return true;
}

static bool parse_sample_csv_line(const std::string &line, GaitTrajectorySample *sample)
{
    std::stringstream stream(line);
    std::string fields[6];
    for (int i = 0; i < 6; i++)
    {
        if (!std::getline(stream, fields[i], ','))
        {
            return false;
        }
    }

    sample->timestamp_ms = atoi(fields[0].c_str());
    sample->phase = fields[1];
    sample->step = atoi(fields[2].c_str());
    sample->channel = atoi(fields[3].c_str());
    sample->pulse_microsec = atoi(fields[4].c_str());
    sample->ticks = atoi(fields[5].c_str());
    return true;
}

bool gait_read_trajectory_csv(const std::string &path,
                              std::vector<GaitTrajectorySample> *samples,
                              std::string *error)
{
    if (samples == 0)
    {
        if (error != 0) { *error = "samples output is null"; }
        return false;
    }
    samples->clear();

    std::ifstream input(path.c_str());
    if (!input)
    {
        if (error != 0) { *error = "failed to open trajectory CSV: " + path; }
        return false;
    }

    std::string line;
    if (!std::getline(input, line))
    {
        if (error != 0) { *error = "trajectory CSV is empty"; }
        return false;
    }

    while (std::getline(input, line))
    {
        if (line.empty())
        {
            continue;
        }

        GaitTrajectorySample sample;
        if (!parse_sample_csv_line(line, &sample))
        {
            if (error != 0) { *error = "failed to parse trajectory CSV line: " + line; }
            return false;
        }
        samples->push_back(sample);
    }

    return true;
}

bool gait_validate_trajectory(const std::vector<GaitTrajectorySample> &samples,
                              int max_delta_microsec,
                              std::string *error)
{
    if (samples.empty() || samples.size() % SERVO_COUNT != 0)
    {
        if (error != 0) { *error = "trajectory must contain complete 12-channel frames"; }
        return false;
    }

    int last_timestamp = -1;
    int last_pulse[SERVO_COUNT];
    bool have_last_pulse[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        last_pulse[i] = 0;
        have_last_pulse[i] = false;
    }

    for (size_t frame_start = 0; frame_start < samples.size(); frame_start += SERVO_COUNT)
    {
        bool seen[SERVO_COUNT] = { false };
        int timestamp = samples[frame_start].timestamp_ms;
        if (timestamp < last_timestamp)
        {
            if (error != 0) { *error = "trajectory timestamps are not monotonic"; }
            return false;
        }
        last_timestamp = timestamp;

        for (int i = 0; i < SERVO_COUNT; i++)
        {
            const GaitTrajectorySample &sample = samples[frame_start + i];
            if (sample.timestamp_ms != timestamp)
            {
                if (error != 0) { *error = "frame contains mixed timestamps"; }
                return false;
            }
            if (!servo_is_valid_index(sample.channel) || seen[sample.channel])
            {
                if (error != 0) { *error = "frame has missing or duplicate channels"; }
                return false;
            }
            seen[sample.channel] = true;

            ServoPulseLimits limits;
            servo_get_pulse_limits(sample.channel, &limits);
            if (sample.pulse_microsec < limits.min_microsec ||
                sample.pulse_microsec > limits.max_microsec)
            {
                if (error != 0) { *error = "trajectory pulse is outside calibration limits"; }
                return false;
            }
            if (sample.ticks != pca9685_pulse_microseconds_to_ticks(sample.pulse_microsec))
            {
                if (error != 0) { *error = "trajectory ticks do not match pulse conversion"; }
                return false;
            }
            if (have_last_pulse[sample.channel] && max_delta_microsec > 0)
            {
                int delta = sample.pulse_microsec > last_pulse[sample.channel]
                    ? sample.pulse_microsec - last_pulse[sample.channel]
                    : last_pulse[sample.channel] - sample.pulse_microsec;
                if (delta > max_delta_microsec)
                {
                    if (error != 0) { *error = "trajectory exceeds max pulse delta"; }
                    return false;
                }
            }

            have_last_pulse[sample.channel] = true;
            last_pulse[sample.channel] = sample.pulse_microsec;
        }
    }

    return true;
}
