#include "semantic_gait.h"

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

static SemanticRobotProfile load_seed_profile()
{
    SemanticRobotProfile profile;
    std::string error;
    assert(semantic_profile_load_json(join_path(REPO_ROOT, "examples/semantic/darkpaw_profile.json"),
                                      &profile,
                                      &error));
    return profile;
}

static void test_seed_profile_and_pose_resolution()
{
    SemanticRobotProfile profile = load_seed_profile();
    std::string poses_dir = join_path(REPO_ROOT, "examples/semantic/poses");
    std::string error;
    GaitPose neutral;
    assert(semantic_pose_resolve(profile, poses_dir, "neutral_stand", &neutral, &error));
    assert(neutral.pulse_microsec[0] == 1040);
    assert(neutral.pulse_microsec[11] == 1550);

    GaitPose lifted;
    assert(semantic_pose_resolve(profile, poses_dir, "front_left_lift", &lifted, &error));
    assert(lifted.pulse_microsec[1] == 380);
    assert(lifted.pulse_microsec[4] == neutral.pulse_microsec[4]);
    assert(lifted.pulse_microsec[10] == neutral.pulse_microsec[10]);
}

static void test_invalid_semantic_target_fails()
{
    SemanticRobotProfile profile = load_seed_profile();
    GaitPose pose;
    gait_pose_init(&pose);
    SemanticTarget target;
    target.legs.push_back("front_left");
    target.axis = "lift";
    target.position = "sideways";
    std::vector<SemanticTarget> targets;
    targets.push_back(target);
    std::string error;
    assert(!semantic_apply_targets(profile, targets, &pose, &error));
}

static void test_seed_semantic_gaits_compile()
{
    SemanticRobotProfile profile = load_seed_profile();
    std::string poses_dir = join_path(REPO_ROOT, "examples/semantic/poses");
    const char *gaits[] = {
        "hold_neutral.json",
        "slow_forward_creep.json",
        "slow_backward_creep.json",
        "slow_rotate_left.json",
        "slow_rotate_right.json",
    };

    for (size_t i = 0; i < sizeof(gaits) / sizeof(gaits[0]); i++)
    {
        SemanticGaitDefinition definition;
        std::vector<GaitTrajectorySample> samples;
        std::string error;
        assert(semantic_gait_load_json(join_path(REPO_ROOT, "examples/semantic/gaits/" + std::string(gaits[i])),
                                       &definition,
                                       &error));
        assert(semantic_gait_compile_trajectory(definition, profile, poses_dir, 80, &samples, &error));
        assert(gait_validate_trajectory(samples, 80, &error));
        assert(samples.size() >= SERVO_COUNT);
    }
}

int main()
{
    test_seed_profile_and_pose_resolution();
    test_invalid_semantic_target_fails();
    test_seed_semantic_gaits_compile();
    return 0;
}
