#include "teleop_state.h"

static double clamp_double(double value, double min_value, double max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static bool movement_is_stop(TeleopMovementType movement)
{
    return movement == TELEOP_MOVEMENT_STOP;
}

static void enter_stop(TeleopState *state, TeleopRunState run_state, TeleopSafetyState safety_state)
{
    state->commanded_movement = TELEOP_MOVEMENT_STOP;
    state->pending_movement = TELEOP_MOVEMENT_STOP;
    state->active_movement = TELEOP_MOVEMENT_STOP;
    state->run_state = run_state;
    state->safety_state = safety_state;
    state->transition_complete_ms = 0;
}

static void request_movement(TeleopState *state, TeleopMovementType movement, uint64_t now_ms)
{
    state->commanded_movement = movement;
    state->safety_state = TELEOP_SAFETY_READY;

    if (movement_is_stop(movement))
    {
        enter_stop(state, TELEOP_RUN_STOPPING, TELEOP_SAFETY_READY);
        return;
    }

    if (state->active_movement == movement && state->pending_movement == TELEOP_MOVEMENT_STOP)
    {
        state->run_state = TELEOP_RUN_RUNNING;
        return;
    }
    if (state->pending_movement == movement)
    {
        return;
    }

    state->pending_movement = movement;
    state->active_movement = TELEOP_MOVEMENT_STOP;
    state->transition_complete_ms = now_ms + state->config.neutral_transition_ms;
    state->run_state = state->run_state == TELEOP_RUN_STOPPED ? TELEOP_RUN_STARTING : TELEOP_RUN_TRANSITIONING;
}

TeleopStateConfig teleop_state_default_config()
{
    TeleopStateConfig config;
    config.command_timeout_ms = 300;
    config.neutral_transition_ms = 600;
    config.min_speed_scale = 0.10;
    config.max_speed_scale = 1.00;
    return config;
}

void teleop_state_init(TeleopState *state, const TeleopStateConfig *config)
{
    state->config = config != 0 ? *config : teleop_state_default_config();
    state->last_sequence_id = 0;
    state->last_receive_ms = 0;
    state->transition_complete_ms = 0;
    state->commanded_movement = TELEOP_MOVEMENT_STOP;
    state->active_movement = TELEOP_MOVEMENT_STOP;
    state->pending_movement = TELEOP_MOVEMENT_STOP;
    state->speed_scale = state->config.min_speed_scale;
    state->run_state = TELEOP_RUN_STOPPED;
    state->safety_state = TELEOP_SAFETY_DISABLED;
    state->accepted_count = 0;
    state->rejected_count = 0;
}

std::string teleop_run_state_to_string(TeleopRunState state)
{
    switch (state)
    {
        case TELEOP_RUN_STOPPED:
            return "stopped";
        case TELEOP_RUN_STARTING:
            return "starting";
        case TELEOP_RUN_RUNNING:
            return "running";
        case TELEOP_RUN_TRANSITIONING:
            return "transitioning";
        case TELEOP_RUN_STOPPING:
            return "stopping";
        case TELEOP_RUN_ESTOPPED:
            return "estopped";
    }
    return "stopped";
}

std::string teleop_safety_state_to_string(TeleopSafetyState state)
{
    switch (state)
    {
        case TELEOP_SAFETY_READY:
            return "ready";
        case TELEOP_SAFETY_DISABLED:
            return "disabled";
        case TELEOP_SAFETY_STALE:
            return "stale";
        case TELEOP_SAFETY_ESTOP:
            return "estop";
        case TELEOP_SAFETY_INVALID:
            return "invalid";
    }
    return "invalid";
}

bool teleop_state_apply_intent(TeleopState *state,
                               const TeleopIntent &intent,
                               uint64_t receive_time_ms,
                               std::string *error)
{
    if (state == 0)
    {
        if (error != 0) { *error = "teleop state is null"; }
        return false;
    }
    if (!teleop_intent_validate(intent, error))
    {
        state->rejected_count++;
        state->safety_state = TELEOP_SAFETY_INVALID;
        return false;
    }
    if (state->last_sequence_id != 0 && intent.sequence_id <= state->last_sequence_id)
    {
        state->rejected_count++;
        state->safety_state = TELEOP_SAFETY_STALE;
        if (error != 0) { *error = "teleop intent sequence is stale or repeated"; }
        return false;
    }

    state->last_sequence_id = intent.sequence_id;
    state->last_receive_ms = receive_time_ms;
    state->accepted_count++;
    state->speed_scale = clamp_double(intent.speed_scale,
                                      state->config.min_speed_scale,
                                      state->config.max_speed_scale);

    if (intent.estop)
    {
        enter_stop(state, TELEOP_RUN_ESTOPPED, TELEOP_SAFETY_ESTOP);
        return true;
    }
    if (!intent.enabled)
    {
        enter_stop(state, TELEOP_RUN_STOPPED, TELEOP_SAFETY_DISABLED);
        return true;
    }

    request_movement(state, intent.movement, receive_time_ms);
    return true;
}

void teleop_state_tick(TeleopState *state, uint64_t now_ms)
{
    if (state == 0)
    {
        return;
    }
    if (state->last_receive_ms != 0 &&
        now_ms > state->last_receive_ms &&
        now_ms - state->last_receive_ms > state->config.command_timeout_ms)
    {
        if (!movement_is_stop(state->active_movement) || !movement_is_stop(state->pending_movement))
        {
            enter_stop(state, TELEOP_RUN_STOPPING, TELEOP_SAFETY_STALE);
        }
        return;
    }

    if (!movement_is_stop(state->pending_movement) &&
        state->transition_complete_ms != 0 &&
        now_ms >= state->transition_complete_ms)
    {
        state->active_movement = state->pending_movement;
        state->pending_movement = TELEOP_MOVEMENT_STOP;
        state->transition_complete_ms = 0;
        state->run_state = TELEOP_RUN_RUNNING;
        state->safety_state = TELEOP_SAFETY_READY;
    }
    else if (state->run_state == TELEOP_RUN_STOPPING)
    {
        state->run_state = TELEOP_RUN_STOPPED;
    }
}

TeleopTelemetry teleop_state_make_telemetry(const TeleopState &state,
                                            uint64_t now_ms,
                                            bool execute_enabled)
{
    TeleopTelemetry telemetry;
    telemetry.timestamp_ms = now_ms;
    telemetry.last_sequence_id = state.last_sequence_id;
    telemetry.commanded_movement = state.commanded_movement;
    telemetry.active_movement = state.active_movement;
    telemetry.safety_state = teleop_safety_state_to_string(state.safety_state);
    telemetry.run_state = teleop_run_state_to_string(state.run_state);
    telemetry.accepted_count = state.accepted_count;
    telemetry.rejected_count = state.rejected_count;
    telemetry.execute_enabled = execute_enabled;
    return telemetry;
}
