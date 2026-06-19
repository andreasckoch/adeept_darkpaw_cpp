#include "gait_definition.h"
#include "gait_pose.h"
#include "gait_trajectory.h"
#include "pca9685.h"

#include <assert.h>
#include <fstream>
#include <sys/stat.h>

static const char *VALID_POSE_JSON =
    "{"
    "\"name\":\"neutral_stand\","
    "\"description\":\"test pose\","
    "\"servos\":["
    "{\"channel\":0,\"pulse_microsec\":1040},"
    "{\"channel\":1,\"pulse_microsec\":940},"
    "{\"channel\":2,\"pulse_microsec\":940},"
    "{\"channel\":3,\"pulse_microsec\":1040},"
    "{\"channel\":4,\"pulse_microsec\":890},"
    "{\"channel\":5,\"pulse_microsec\":1000},"
    "{\"channel\":6,\"pulse_microsec\":890},"
    "{\"channel\":7,\"pulse_microsec\":940},"
    "{\"channel\":8,\"pulse_microsec\":1100},"
    "{\"channel\":9,\"pulse_microsec\":1140},"
    "{\"channel\":10,\"pulse_microsec\":1075},"
    "{\"channel\":11,\"pulse_microsec\":1550}"
    "]"
    "}";

static const char *VALID_GAIT_JSON =
    "{"
    "\"name\":\"hold_neutral\","
    "\"description\":\"test gait\","
    "\"phases\":["
    "{\"name\":\"hold\",\"from\":\"neutral_stand\",\"to\":\"neutral_stand\",\"duration_ms\":1000,\"steps\":10}"
    "]"
    "}";

static void write_file(const std::string &path, const std::string &content)
{
    std::ofstream output(path.c_str());
    output << content;
}

static void test_pose_parsing()
{
    GaitPose pose;
    std::string error;
    assert(gait_pose_parse_json(VALID_POSE_JSON, &pose, &error));
    assert(pose.name == "neutral_stand");
    assert(pose.channel_present[0]);
    assert(pose.pulse_microsec[11] == 1550);
}

static void test_pose_missing_channel_fails()
{
    const char *json =
        "{"
        "\"name\":\"bad_pose\","
        "\"servos\":[{\"channel\":0,\"pulse_microsec\":1040}]"
        "}";
    GaitPose pose;
    std::string error;
    assert(!gait_pose_parse_json(json, &pose, &error));
}

static void test_pose_out_of_range_fails()
{
    std::string json = VALID_POSE_JSON;
    size_t pos = json.find("\"channel\":0,\"pulse_microsec\":1040");
    json.replace(pos, 32, "\"channel\":0,\"pulse_microsec\":9999");
    GaitPose pose;
    std::string error;
    assert(!gait_pose_parse_json(json, &pose, &error));
}

static void test_gait_parsing()
{
    GaitDefinition definition;
    std::string error;
    assert(gait_definition_parse_json(VALID_GAIT_JSON, &definition, &error));
    assert(definition.name == "hold_neutral");
    assert(definition.phases.size() == 1);
    assert(definition.phases[0].steps == 10);
}

static void test_compile_and_validate_trajectory()
{
    mkdir("test_poses", 0777);
    write_file("test_poses/neutral_stand.json", VALID_POSE_JSON);

    GaitDefinition definition;
    std::string error;
    assert(gait_definition_parse_json(VALID_GAIT_JSON, &definition, &error));

    std::vector<GaitTrajectorySample> samples;
    assert(gait_compile_trajectory(definition, "test_poses", 200, &samples, &error));
    assert(samples.size() == 11 * SERVO_COUNT);
    assert(gait_validate_trajectory(samples, 200, &error));
    assert(samples[0].ticks == pca9685_pulse_microseconds_to_ticks(samples[0].pulse_microsec));

    assert(gait_write_trajectory_csv("hold_neutral.csv", samples, &error));
    std::vector<GaitTrajectorySample> readback;
    assert(gait_read_trajectory_csv("hold_neutral.csv", &readback, &error));
    assert(gait_validate_trajectory(readback, 200, &error));
    assert(readback.size() == samples.size());
}

static void test_missing_pose_reference_fails()
{
    GaitDefinition definition;
    std::string error;
    assert(gait_definition_parse_json(VALID_GAIT_JSON, &definition, &error));
    std::vector<GaitTrajectorySample> samples;
    assert(!gait_compile_trajectory(definition, "missing_pose_dir", 200, &samples, &error));
}

int main()
{
    test_pose_parsing();
    test_pose_missing_channel_fails();
    test_pose_out_of_range_fails();
    test_gait_parsing();
    test_compile_and_validate_trajectory();
    test_missing_pose_reference_fails();
    return 0;
}
