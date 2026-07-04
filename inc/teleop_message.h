#ifndef TELEOP_MESSAGE_H_
#define TELEOP_MESSAGE_H_

#include <stdint.h>
#include <string>

enum TeleopMovementType
{
    TELEOP_MOVEMENT_STOP = 0,
    TELEOP_MOVEMENT_FORWARD,
    TELEOP_MOVEMENT_BACKWARD,
    TELEOP_MOVEMENT_LEFT,
    TELEOP_MOVEMENT_RIGHT,
    TELEOP_MOVEMENT_ROTATE_LEFT,
    TELEOP_MOVEMENT_ROTATE_RIGHT
};

struct TeleopIntent
{
    uint32_t sequence_id;
    uint64_t timestamp_ms;
    TeleopMovementType movement;
    double speed_scale;
    bool enabled;
    bool estop;
};

struct TeleopTelemetry
{
    uint64_t timestamp_ms;
    uint32_t last_sequence_id;
    TeleopMovementType commanded_movement;
    TeleopMovementType active_movement;
    std::string safety_state;
    std::string run_state;
    uint32_t accepted_count;
    uint32_t rejected_count;
    bool execute_enabled;
};

void teleop_intent_init(TeleopIntent *intent);
bool teleop_movement_from_string(const std::string &value, TeleopMovementType *movement);
std::string teleop_movement_to_string(TeleopMovementType movement);
bool teleop_movement_from_key(char key, TeleopMovementType *movement, bool *estop, bool *enabled);
bool teleop_intent_validate(const TeleopIntent &intent, std::string *error);
std::string teleop_intent_serialize(const TeleopIntent &intent);
bool teleop_intent_parse(const std::string &packet, TeleopIntent *intent, std::string *error);
std::string teleop_telemetry_serialize(const TeleopTelemetry &telemetry);

#endif /* TELEOP_MESSAGE_H_ */
