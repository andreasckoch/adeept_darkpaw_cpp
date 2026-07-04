#ifndef TELEOP_STATE_H_
#define TELEOP_STATE_H_

#include "teleop_message.h"

#include <stdint.h>
#include <string>

enum TeleopRunState
{
    TELEOP_RUN_STOPPED = 0,
    TELEOP_RUN_STARTING,
    TELEOP_RUN_RUNNING,
    TELEOP_RUN_TRANSITIONING,
    TELEOP_RUN_STOPPING,
    TELEOP_RUN_ESTOPPED
};

enum TeleopSafetyState
{
    TELEOP_SAFETY_READY = 0,
    TELEOP_SAFETY_DISABLED,
    TELEOP_SAFETY_STALE,
    TELEOP_SAFETY_ESTOP,
    TELEOP_SAFETY_INVALID
};

struct TeleopStateConfig
{
    uint64_t command_timeout_ms;
    uint64_t neutral_transition_ms;
    double min_speed_scale;
    double max_speed_scale;
};

struct TeleopState
{
    TeleopStateConfig config;
    uint32_t last_sequence_id;
    uint64_t last_receive_ms;
    uint64_t transition_complete_ms;
    TeleopMovementType commanded_movement;
    TeleopMovementType active_movement;
    TeleopMovementType pending_movement;
    double speed_scale;
    TeleopRunState run_state;
    TeleopSafetyState safety_state;
    uint32_t accepted_count;
    uint32_t rejected_count;
};

TeleopStateConfig teleop_state_default_config();
void teleop_state_init(TeleopState *state, const TeleopStateConfig *config);
std::string teleop_run_state_to_string(TeleopRunState state);
std::string teleop_safety_state_to_string(TeleopSafetyState state);
bool teleop_state_apply_intent(TeleopState *state,
                               const TeleopIntent &intent,
                               uint64_t receive_time_ms,
                               std::string *error);
void teleop_state_tick(TeleopState *state, uint64_t now_ms);
TeleopTelemetry teleop_state_make_telemetry(const TeleopState &state,
                                            uint64_t now_ms,
                                            bool execute_enabled);

#endif /* TELEOP_STATE_H_ */
