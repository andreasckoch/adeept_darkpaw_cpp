#include "gait_definition.h"
#include "gait_pose.h"
#include "gait_trajectory.h"

#include <assert.h>
#include <string>
#include <vector>

#ifndef REPO_ROOT
#define REPO_ROOT "."
#endif

static std::string join_path(const std::string &left, const std::string &right)
{
    if (left.empty() || left[left.size() - 1] == '/')
    {
        return left + right;
    }
    return left + "/" + right;
}

static void test_seed_neutral_pose_loads()
{
    GaitPose pose;
    std::string error;
    assert(gait_pose_load_json(join_path(REPO_ROOT, "examples/poses_old/neutral_stand.json"),
                               &pose,
                               &error));
    assert(pose.name == "neutral_stand");
}

static void test_seed_gait_compiles(const std::string &gait_file)
{
    std::string root = REPO_ROOT;
    std::string poses_dir = join_path(root, "examples/poses_old");
    std::string gait_path = join_path(root, "examples/gaits_old/" + gait_file);

    GaitDefinition definition;
    std::vector<GaitTrajectorySample> samples;
    std::string error;
    assert(gait_definition_load_json(gait_path, &definition, &error));
    assert(gait_compile_trajectory(definition, poses_dir, 80, &samples, &error));
    assert(gait_validate_trajectory(samples, 80, &error));
    assert(samples.size() >= SERVO_COUNT);
}

int main()
{
    test_seed_neutral_pose_loads();

    const char *gaits[] = {
        "hold_neutral.json",
        "slow_forward_creep.json",
        "slow_backward_creep.json",
        "slow_rotate_left.json",
        "slow_rotate_right.json",
    };

    for (size_t i = 0; i < sizeof(gaits) / sizeof(gaits[0]); i++)
    {
        test_seed_gait_compiles(gaits[i]);
    }

    return 0;
}
