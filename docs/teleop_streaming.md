# Keyboard Teleop Streaming

This is the first control-streaming path for keyboard and later Steam Deck
teleoperation. It streams high-level movement intent, not servo pulse targets.
The robot node remains responsible for watchdogs, movement transitions, gait
selection, and calibrated actuator bounds.

## Packet Split

The current control stream is a small UDP intent packet:

```text
SPIDER_INTENT_V1 sequence_id timestamp_ms movement speed_scale enabled estop
```

Supported movements are:

- `forward`
- `backward`
- `left`
- `right`
- `rotate_left`
- `rotate_right`
- `stop`

The robot replies to the sender with telemetry:

```text
SPIDER_TELEMETRY_V1 timestamp_ms last_sequence_id commanded_movement active_movement safety_state run_state accepted_count rejected_count execute_enabled
```

Camera, IMU, depth, and richer actuator telemetry should use separate streams.
Do not mix video reliability or bandwidth assumptions into this control packet.

## Safety Behavior

The robot node rejects malformed and repeated sequence IDs. If command packets
stop arriving for the watchdog window, it enters `stale` safety state and stops
the active loop. Movement changes pass through a neutral transition window before
the next loop becomes active.

Current run states:

- `stopped`
- `starting`
- `running`
- `transitioning`
- `stopping`
- `estopped`

This is still open-loop scripted motion. It is not closed-loop balance control.
Keep the robot supported for first hardware tests.

## Dry-Run Workflow

Build first:

```bash
cmake -S . -B build
cmake --build build
```

Start the robot-side receiver in dry-run mode on the Pi:

```bash
scripts/run_teleop_robot_node.sh
```

Start the keyboard client from the Mac, replacing the host with the Pi address:

```bash
scripts/run_teleop_keyboard.sh --host 192.168.1.50
```

Keyboard mapping:

- `W`: forward
- `S`: backward
- `A`: left
- `D`: right
- `Q`: rotate left
- `E`: rotate right
- space: stop
- `X`: e-stop

The terminal client has no true key-release events. It streams stop when no key
has been seen for `--hold-ms`.

## Hardware Execution

Hardware movement remains explicit:

```bash
sudo scripts/run_teleop_robot_node.sh --execute
```

Use dry-run first, keep the robot supported, and keep power reachable. The node
opens pigpio and the PCA9685 only when `--execute` is present.

## Steam Deck Path

The keyboard client and movement intent schema are deliberately separate from
robot-side execution. A Steam Deck client can reuse the same packet format and
replace terminal input with SDL game controller input:

- left stick maps to `forward/backward/left/right` intent and speed scale
- right stick X maps to `rotate_left/rotate_right`
- an enable button controls `enabled`
- a dedicated button sends `estop`

The next transport iteration can replace this UDP packet with ROS 2 messages or
add a parallel telemetry/video stack without changing the low-level servo path.
