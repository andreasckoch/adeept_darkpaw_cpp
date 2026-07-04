#include "teleop_message.h"

#include <math.h>
#include <sstream>

static const char *INTENT_MAGIC = "SPIDER_INTENT_V1";
static const char *TELEMETRY_MAGIC = "SPIDER_TELEMETRY_V1";

void teleop_intent_init(TeleopIntent *intent)
{
    intent->sequence_id = 0;
    intent->timestamp_ms = 0;
    intent->movement = TELEOP_MOVEMENT_STOP;
    intent->speed_scale = 0.0;
    intent->enabled = false;
    intent->estop = false;
}

bool teleop_movement_from_string(const std::string &value, TeleopMovementType *movement)
{
    if (movement == 0)
    {
        return false;
    }
    if (value == "stop")
    {
        *movement = TELEOP_MOVEMENT_STOP;
    }
    else if (value == "forward")
    {
        *movement = TELEOP_MOVEMENT_FORWARD;
    }
    else if (value == "backward")
    {
        *movement = TELEOP_MOVEMENT_BACKWARD;
    }
    else if (value == "left")
    {
        *movement = TELEOP_MOVEMENT_LEFT;
    }
    else if (value == "right")
    {
        *movement = TELEOP_MOVEMENT_RIGHT;
    }
    else if (value == "rotate_left")
    {
        *movement = TELEOP_MOVEMENT_ROTATE_LEFT;
    }
    else if (value == "rotate_right")
    {
        *movement = TELEOP_MOVEMENT_ROTATE_RIGHT;
    }
    else
    {
        return false;
    }
    return true;
}

std::string teleop_movement_to_string(TeleopMovementType movement)
{
    switch (movement)
    {
        case TELEOP_MOVEMENT_STOP:
            return "stop";
        case TELEOP_MOVEMENT_FORWARD:
            return "forward";
        case TELEOP_MOVEMENT_BACKWARD:
            return "backward";
        case TELEOP_MOVEMENT_LEFT:
            return "left";
        case TELEOP_MOVEMENT_RIGHT:
            return "right";
        case TELEOP_MOVEMENT_ROTATE_LEFT:
            return "rotate_left";
        case TELEOP_MOVEMENT_ROTATE_RIGHT:
            return "rotate_right";
    }
    return "stop";
}

bool teleop_movement_from_key(char key, TeleopMovementType *movement, bool *estop, bool *enabled)
{
    if (movement == 0 || estop == 0 || enabled == 0)
    {
        return false;
    }
    *estop = false;
    *enabled = true;
    switch (key)
    {
        case 'w':
        case 'W':
            *movement = TELEOP_MOVEMENT_FORWARD;
            return true;
        case 's':
        case 'S':
            *movement = TELEOP_MOVEMENT_BACKWARD;
            return true;
        case 'a':
        case 'A':
            *movement = TELEOP_MOVEMENT_LEFT;
            return true;
        case 'd':
        case 'D':
            *movement = TELEOP_MOVEMENT_RIGHT;
            return true;
        case 'q':
        case 'Q':
            *movement = TELEOP_MOVEMENT_ROTATE_LEFT;
            return true;
        case 'e':
        case 'E':
            *movement = TELEOP_MOVEMENT_ROTATE_RIGHT;
            return true;
        case ' ':
            *movement = TELEOP_MOVEMENT_STOP;
            return true;
        case 'x':
        case 'X':
            *movement = TELEOP_MOVEMENT_STOP;
            *estop = true;
            return true;
        default:
            *movement = TELEOP_MOVEMENT_STOP;
            *enabled = false;
            return false;
    }
}

bool teleop_intent_validate(const TeleopIntent &intent, std::string *error)
{
    if (intent.movement < TELEOP_MOVEMENT_STOP || intent.movement > TELEOP_MOVEMENT_ROTATE_RIGHT)
    {
        if (error != 0) { *error = "teleop intent contains an unknown movement"; }
        return false;
    }
    if (!isfinite(intent.speed_scale) || intent.speed_scale < 0.0 || intent.speed_scale > 1.0)
    {
        if (error != 0) { *error = "teleop intent speed_scale must be finite and between 0 and 1"; }
        return false;
    }
    if (intent.timestamp_ms == 0)
    {
        if (error != 0) { *error = "teleop intent timestamp_ms must be non-zero"; }
        return false;
    }
    return true;
}

std::string teleop_intent_serialize(const TeleopIntent &intent)
{
    std::ostringstream stream;
    stream << INTENT_MAGIC << " "
           << intent.sequence_id << " "
           << intent.timestamp_ms << " "
           << teleop_movement_to_string(intent.movement) << " "
           << intent.speed_scale << " "
           << (intent.enabled ? 1 : 0) << " "
           << (intent.estop ? 1 : 0);
    return stream.str();
}

bool teleop_intent_parse(const std::string &packet, TeleopIntent *intent, std::string *error)
{
    if (intent == 0)
    {
        if (error != 0) { *error = "teleop intent output is null"; }
        return false;
    }

    std::istringstream stream(packet);
    std::string magic;
    std::string movement_name;
    int enabled = 0;
    int estop = 0;
    TeleopIntent parsed;
    teleop_intent_init(&parsed);

    stream >> magic
           >> parsed.sequence_id
           >> parsed.timestamp_ms
           >> movement_name
           >> parsed.speed_scale
           >> enabled
           >> estop;
    if (!stream || magic != INTENT_MAGIC)
    {
        if (error != 0) { *error = "teleop packet is not a valid SPIDER_INTENT_V1 message"; }
        return false;
    }
    if (!teleop_movement_from_string(movement_name, &parsed.movement))
    {
        if (error != 0) { *error = "teleop packet contains an unknown movement"; }
        return false;
    }
    parsed.enabled = enabled != 0;
    parsed.estop = estop != 0;
    if (!teleop_intent_validate(parsed, error))
    {
        return false;
    }

    *intent = parsed;
    return true;
}

std::string teleop_telemetry_serialize(const TeleopTelemetry &telemetry)
{
    std::ostringstream stream;
    stream << TELEMETRY_MAGIC << " "
           << telemetry.timestamp_ms << " "
           << telemetry.last_sequence_id << " "
           << teleop_movement_to_string(telemetry.commanded_movement) << " "
           << teleop_movement_to_string(telemetry.active_movement) << " "
           << telemetry.safety_state << " "
           << telemetry.run_state << " "
           << telemetry.accepted_count << " "
           << telemetry.rejected_count << " "
           << (telemetry.execute_enabled ? 1 : 0);
    return stream.str();
}
