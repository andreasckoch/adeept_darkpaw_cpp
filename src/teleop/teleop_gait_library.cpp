#include "teleop_gait_library.h"

static SemanticTarget target(const char *leg_a,
                             const char *leg_b,
                             const char *axis,
                             const char *position)
{
    SemanticTarget result;
    result.legs.push_back(leg_a);
    if (leg_b != 0)
    {
        result.legs.push_back(leg_b);
    }
    result.axis = axis;
    result.position = position;
    return result;
}

static void add_phase(SemanticGaitDefinition *definition,
                      const std::string &name,
                      int duration_ms,
                      int steps,
                      const std::vector<SemanticTarget> &targets)
{
    SemanticGaitPhase phase;
    phase.name = name;
    phase.duration_ms = duration_ms;
    phase.steps = steps;
    phase.targets = targets;
    definition->phases.push_back(phase);
}

static std::vector<SemanticTarget> diagonal_targets(const char *lift_a,
                                                   const char *lift_b,
                                                   const char *swing_fore_aft,
                                                   const char *support_fore_aft)
{
    std::vector<SemanticTarget> targets;
    targets.push_back(target(lift_a, lift_b, "fore_aft", swing_fore_aft));
    if (lift_a == std::string("front_left"))
    {
        targets.push_back(target("front_right", "rear_left", "fore_aft", support_fore_aft));
    }
    else
    {
        targets.push_back(target("front_left", "rear_right", "fore_aft", support_fore_aft));
    }
    return targets;
}

static void add_diagonal_creep(SemanticGaitDefinition *definition,
                               const TeleopGaitTiming &timing,
                               const char *swing_fore_aft,
                               const char *support_fore_aft,
                               const std::string &prefix)
{
    std::vector<SemanticTarget> a_lift;
    a_lift.push_back(target("front_left", "rear_right", "lift", "up"));
    add_phase(definition, prefix + "_a_lift", timing.phase_duration_ms, timing.phase_steps, a_lift);

    add_phase(definition,
              prefix + "_a_swing",
              timing.phase_duration_ms,
              timing.phase_steps,
              diagonal_targets("front_left", "rear_right", swing_fore_aft, support_fore_aft));

    std::vector<SemanticTarget> a_place;
    a_place.push_back(target("front_left", "rear_right", "lift", "down"));
    add_phase(definition, prefix + "_a_place", timing.phase_duration_ms, timing.phase_steps, a_place);

    std::vector<SemanticTarget> b_lift;
    b_lift.push_back(target("front_right", "rear_left", "lift", "up"));
    add_phase(definition, prefix + "_b_lift", timing.phase_duration_ms, timing.phase_steps, b_lift);
    
    add_phase(definition,
              prefix + "_b_swing",
              timing.phase_duration_ms,
              timing.phase_steps,
              diagonal_targets("front_right", "rear_left", swing_fore_aft, support_fore_aft));

    std::vector<SemanticTarget> b_place;
    b_place.push_back(target("front_right", "rear_left", "lift", "down"));
    add_phase(definition, prefix + "_b_place", timing.phase_duration_ms, timing.phase_steps, b_place);
}

static void add_strafe(SemanticGaitDefinition *definition,
                       const TeleopGaitTiming &timing,
                       const char *left_stance,
                       const char *right_stance,
                       const std::string &prefix)
{
    std::vector<SemanticTarget> a_swing;
    a_swing.push_back(target("front_left", "rear_right", "lift", "up"));
    a_swing.push_back(target("front_left", "front_right", "stance", left_stance));
    a_swing.push_back(target("rear_left", "rear_right", "stance", right_stance));
    add_phase(definition, prefix + "_a_swing", timing.phase_duration_ms, timing.phase_steps, a_swing);

    std::vector<SemanticTarget> a_place;
    a_place.push_back(target("front_left", "rear_right", "lift", "down"));
    add_phase(definition, prefix + "_a_place", timing.phase_duration_ms, timing.phase_steps, a_place);

    std::vector<SemanticTarget> b_swing;
    b_swing.push_back(target("front_right", "rear_left", "lift", "up"));
    b_swing.push_back(target("rear_right", "rear_left", "stance", left_stance));
    b_swing.push_back(target("front_right", "front_left", "stance", right_stance));
    add_phase(definition, prefix + "_b_swing", timing.phase_duration_ms, timing.phase_steps, b_swing);

    std::vector<SemanticTarget> b_place;
    b_place.push_back(target("front_right", "rear_left", "lift", "down"));
    add_phase(definition, prefix + "_b_place", timing.phase_duration_ms, timing.phase_steps, b_place);
}

static void add_rotation(SemanticGaitDefinition *definition,
                         const TeleopGaitTiming &timing,
                         const char *fore_aft_1, // back
                         const char *fore_aft_2, // front
                         const std::string &prefix)
{
    for (int i = 0; i < 2; i++)
    {
        std::vector<SemanticTarget> a_rotate;
        a_rotate.push_back(target("front_right", "rear_right", "fore_aft", fore_aft_1));
        a_rotate.push_back(target("rear_left", "front_left", "fore_aft", fore_aft_2));
        add_phase(definition, prefix + "_a_rotate", timing.phase_duration_ms, timing.phase_steps, a_rotate);
        
        std::vector<SemanticTarget> a_lift_1;
        a_lift_1.push_back(target("front_left", "rear_right", "lift", "up"));
        // a_lift_1.push_back(target("front_left", "rear_right", "fore_aft", "neutral"));
        a_lift_1.push_back(target("front_left", 0, "fore_aft", fore_aft_1));
        a_lift_1.push_back(target("rear_right", 0, "fore_aft", fore_aft_2));
        add_phase(definition, prefix + "_a_lift_1", timing.phase_duration_ms, timing.phase_steps, a_lift_1);

        std::vector<SemanticTarget> a_put_1;
        a_put_1.push_back(target("front_left", "rear_right", "lift", "neutral"));
        add_phase(definition, prefix + "_a_put_1", timing.phase_duration_ms, timing.phase_steps, a_put_1);

        std::vector<SemanticTarget> a_lift_2;
        a_lift_2.push_back(target("front_right", "rear_left", "lift", "up"));
        // a_lift_2.push_back(target("front_right", "rear_left", "fore_aft", "neutral"));
        a_lift_2.push_back(target("front_right", 0, "fore_aft", fore_aft_2));
        a_lift_2.push_back(target("rear_left", 0, "fore_aft", fore_aft_1));
        add_phase(definition, prefix + "_a_lift_2", timing.phase_duration_ms, timing.phase_steps, a_lift_2);

        std::vector<SemanticTarget> a_put_2;
        a_put_2.push_back(target("front_right", "rear_left", "lift", "neutral"));
        add_phase(definition, prefix + "_a_put_2", timing.phase_duration_ms, timing.phase_steps, a_put_2);
    }
}

TeleopGaitTiming teleop_gait_default_timing()
{
    TeleopGaitTiming timing;
    timing.phase_duration_ms = 420;
    timing.phase_steps = 24;
    timing.max_delta_microsec = 80;
    return timing;
}

bool teleop_gait_build_definition(TeleopMovementType movement,
                                  const TeleopGaitTiming &timing,
                                  SemanticGaitDefinition *definition,
                                  std::string *error)
{
    if (definition == 0 || timing.phase_duration_ms <= 0 || timing.phase_steps <= 0)
    {
        if (error != 0) { *error = "invalid teleop gait timing or output"; }
        return false;
    }

    semantic_gait_init(definition);
    definition->name = "teleop_" + teleop_movement_to_string(movement) + "_loop";
    definition->description = "Built-in semantic loop for keyboard/Steam Deck style teleop intent.";
    definition->initial_pose = "neutral_stand";

    switch (movement)
    {
        case TELEOP_MOVEMENT_FORWARD:
            add_diagonal_creep(definition, timing, "front", "back", "forward");
            break;
        case TELEOP_MOVEMENT_BACKWARD:
            add_diagonal_creep(definition, timing, "back", "front", "backward");
            break;
        case TELEOP_MOVEMENT_LEFT:
            add_strafe(definition, timing, "wide", "close", "left");
            break;
        case TELEOP_MOVEMENT_RIGHT:
            add_strafe(definition, timing, "close", "wide", "right");
            break;
        case TELEOP_MOVEMENT_ROTATE_LEFT:
            add_rotation(definition, timing, "back", "front", "rotate_left");
            break;
        case TELEOP_MOVEMENT_ROTATE_RIGHT:
            add_rotation(definition, timing, "front", "back", "rotate_right");
            break;
        case TELEOP_MOVEMENT_STOP:
            if (error != 0) { *error = "stop does not have a movement loop"; }
            return false;
    }

    if (!semantic_gait_validate(*definition, error))
    {
        return false;
    }
    return true;
}

bool teleop_gait_compile_loop(TeleopMovementType movement,
                              const SemanticRobotProfile &profile,
                              const std::string &poses_dir,
                              const TeleopGaitTiming &timing,
                              std::vector<GaitTrajectorySample> *samples,
                              std::string *error)
{
    SemanticGaitDefinition definition;
    if (!teleop_gait_build_definition(movement, timing, &definition, error))
    {
        return false;
    }
    return semantic_gait_compile_trajectory(definition,
                                            profile,
                                            poses_dir,
                                            timing.max_delta_microsec,
                                            samples,
                                            error);
}
