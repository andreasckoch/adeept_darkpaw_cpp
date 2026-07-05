#include "teleop_gait_library.h"
#include "teleop_state.h"

#include <assert.h>

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

static void test_key_mapping_and_packet_roundtrip()
{
    TeleopMovementType movement = TELEOP_MOVEMENT_STOP;
    bool estop = false;
    bool enabled = false;
    assert(teleop_movement_from_key('w', &movement, &estop, &enabled));
    assert(movement == TELEOP_MOVEMENT_FORWARD);
    assert(enabled);
    assert(!estop);

    assert(teleop_movement_from_key('Q', &movement, &estop, &enabled));
    assert(movement == TELEOP_MOVEMENT_ROTATE_LEFT);

    assert(teleop_movement_from_key('x', &movement, &estop, &enabled));
    assert(movement == TELEOP_MOVEMENT_STOP);
    assert(estop);

    TeleopIntent intent;
    teleop_intent_init(&intent);
    intent.sequence_id = 7;
    intent.timestamp_ms = 1234;
    intent.movement = TELEOP_MOVEMENT_RIGHT;
    intent.speed_scale = 0.4;
    intent.enabled = true;
    intent.estop = false;

    std::string packet = teleop_intent_serialize(intent);
    TeleopIntent parsed;
    std::string error;
    assert(teleop_intent_parse(packet, &parsed, &error));
    assert(parsed.sequence_id == intent.sequence_id);
    assert(parsed.timestamp_ms == intent.timestamp_ms);
    assert(parsed.movement == intent.movement);
    assert(parsed.enabled == intent.enabled);
    assert(parsed.estop == intent.estop);
}

static void test_state_machine_transitions_and_timeout()
{
    TeleopStateConfig config = teleop_state_default_config();
    config.command_timeout_ms = 100;
    config.neutral_transition_ms = 50;

    TeleopState state;
    teleop_state_init(&state, &config);

    TeleopIntent intent;
    teleop_intent_init(&intent);
    intent.sequence_id = 1;
    intent.timestamp_ms = 1000;
    intent.movement = TELEOP_MOVEMENT_FORWARD;
    intent.speed_scale = 0.05;
    intent.enabled = true;

    std::string error;
    assert(teleop_state_apply_intent(&state, intent, 1000, &error));
    assert(state.active_movement == TELEOP_MOVEMENT_STOP);
    assert(state.pending_movement == TELEOP_MOVEMENT_FORWARD);
    assert(state.speed_scale == config.min_speed_scale);

    intent.sequence_id = 2;
    intent.timestamp_ms = 1020;
    assert(teleop_state_apply_intent(&state, intent, 1020, &error));
    assert(state.transition_complete_ms == 1050);

    teleop_state_tick(&state, 1050);
    assert(state.active_movement == TELEOP_MOVEMENT_FORWARD);
    assert(state.pending_movement == TELEOP_MOVEMENT_STOP);
    assert(state.run_state == TELEOP_RUN_RUNNING);

    intent.sequence_id = 3;
    intent.timestamp_ms = 1060;
    intent.movement = TELEOP_MOVEMENT_ROTATE_RIGHT;
    intent.speed_scale = 0.8;
    assert(teleop_state_apply_intent(&state, intent, 1060, &error));
    assert(state.active_movement == TELEOP_MOVEMENT_STOP);
    assert(state.pending_movement == TELEOP_MOVEMENT_ROTATE_RIGHT);
    assert(state.run_state == TELEOP_RUN_TRANSITIONING);

    teleop_state_tick(&state, 1110);
    assert(state.active_movement == TELEOP_MOVEMENT_ROTATE_RIGHT);

    assert(!teleop_state_apply_intent(&state, intent, 1120, &error));
    assert(state.rejected_count == 1);

    teleop_state_tick(&state, 1300);
    assert(state.active_movement == TELEOP_MOVEMENT_STOP);
    assert(state.safety_state == TELEOP_SAFETY_STALE);
}

static void test_estop_and_disabled_fail_closed()
{
    TeleopState state;
    teleop_state_init(&state, 0);

    TeleopIntent intent;
    teleop_intent_init(&intent);
    intent.sequence_id = 1;
    intent.timestamp_ms = 100;
    intent.movement = TELEOP_MOVEMENT_LEFT;
    intent.speed_scale = 0.5;
    intent.enabled = true;
    intent.estop = true;

    std::string error;
    assert(teleop_state_apply_intent(&state, intent, 100, &error));
    assert(state.active_movement == TELEOP_MOVEMENT_STOP);
    assert(state.run_state == TELEOP_RUN_ESTOPPED);
    assert(state.safety_state == TELEOP_SAFETY_ESTOP);
}

static void test_builtin_gaits_compile()
{
    SemanticRobotProfile profile;
    std::string error;
    assert(semantic_profile_load_json(join_path(REPO_ROOT, "examples/semantic/darkpaw_profile.json"),
                                      &profile,
                                      &error));

    std::string poses_dir = join_path(REPO_ROOT, "examples/semantic/poses");
    TeleopGaitTiming timing = teleop_gait_default_timing();
    TeleopMovementType movements[] = {
        TELEOP_MOVEMENT_FORWARD,
        TELEOP_MOVEMENT_BACKWARD,
        TELEOP_MOVEMENT_LEFT,
        TELEOP_MOVEMENT_RIGHT,
        TELEOP_MOVEMENT_ROTATE_LEFT,
        TELEOP_MOVEMENT_ROTATE_RIGHT
    };

    for (size_t i = 0; i < sizeof(movements) / sizeof(movements[0]); i++)
    {
        SemanticGaitDefinition definition;
        assert(teleop_gait_build_definition(movements[i], timing, &definition, &error));
        for (size_t phase_index = 0; phase_index < definition.phases.size(); phase_index++)
        {
            assert(definition.phases[phase_index].target_pose.empty());
        }

        std::vector<GaitTrajectorySample> samples;
        assert(teleop_gait_compile_loop(movements[i], profile, poses_dir, timing, &samples, &error));
        assert(gait_validate_trajectory(samples, timing.max_delta_microsec, &error));
        assert(samples.size() >= SERVO_COUNT);
    }
}

int main()
{
    test_key_mapping_and_packet_roundtrip();
    test_state_machine_transitions_and_timeout();
    test_estop_and_disabled_fail_closed();
    test_builtin_gaits_compile();
    return 0;
}
