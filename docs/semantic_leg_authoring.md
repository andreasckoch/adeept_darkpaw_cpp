# Semantic Leg Authoring

The semantic authoring path converts body-relative leg targets into the existing
12-channel CSV trajectory format. It does not change the PCA9685 player or bypass
servo calibration limits.

## Profile

`examples/semantic/darkpaw_profile.json` is the only authoring file that contains
PCA9685 channels and pulse widths. It defines four legs and three joints per leg:

- `fore_aft`: `back`, `neutral`, `front`
- `lift`: `down`, `neutral`, `up`
- `stance`: `close`, `neutral`, `wide`

Names are body-relative. For example, `front` means toward the robot's front, not
"higher PWM". The profile maps each leg's mounted servo orientation to that meaning.

Update the profile only after testing one joint at a time with the robot supported.
Every pulse must remain inside `src/actuation/servo_calibration.cpp` limits. The
checked-in values are seed values, not a claim of hardware-validated gait geometry.

Validate it without hardware access:

```bash
./build/spider_validate_semantic_profile \
  --profile examples/semantic/darkpaw_profile.json
```

Test the three values for a single named joint before using that joint in a gait:

```bash
scripts/test_semantic_joint_on_robot.sh --leg front_left --axis lift
scripts/test_semantic_joint_on_robot.sh --leg front_left --axis lift --execute
```

The test moves `neutral -> down -> neutral -> up -> neutral` for `lift`,
`neutral -> back -> neutral -> front -> neutral` for `fore_aft`, and the analogous
`close/wide` sequence for `stance`. It writes only the resolved joint channel and
returns to profile neutral. Keep the robot supported and servo power reachable.

After reassembly, the all-joint test runs the four standard Darkpaw legs and three
semantic axes serially. Its default 100 us steps, 20 ms step delay, and 250 ms
settle time keep each joint test well below one third of the single-joint tester's
default duration while still commanding both profile endpoints:

```bash
scripts/test_all_semantic_joints_on_robot.sh
scripts/test_all_semantic_joints_on_robot.sh --execute
```

The script expects the standard `front_left`, `rear_left`, `front_right`, and
`rear_right` leg names. Use the single-joint script for a profile with different
leg naming.

For a coordinated complete-robot check, run:

```bash
scripts/test_semantic_robot_movements_on_robot.sh
scripts/test_semantic_robot_movements_on_robot.sh --execute
```

It generates a semantic gait that raises and lowers all legs together, exercises
left/right stance targets through opposing `wide/close` leg pairs, then applies a
profile-level counterbalance sequence before each individual fore/aft sweep. For
example, before testing `front_left`, it raises `rear_left`, moves `front_right`
and `rear_left` down, raises `front_left`, and then restores the support pair to
neutral after the sweep. The default playback scale is 0.40, reducing effective
per-frame delay from 100 ms to about 63 ms versus the previous 0.25 default.
This is a profile and motion-path check, not proof of closed-loop balance or stable
body translation.

## Poses

A semantic pose targets named legs and joint states. A pose with `base` inherits
all unspecified joints from another pose, so only changed movements are written.

```json
{
  "name": "diagonal_a_lift",
  "base": "neutral_stand",
  "targets": [
    { "legs": ["front_left", "rear_right"], "axis": "lift", "position": "up" }
  ]
}
```

Inspect its resolved 12-channel result without opening hardware:

```bash
./build/spider_print_semantic_pose \
  --profile examples/semantic/darkpaw_profile.json \
  --poses examples/semantic/poses \
  --pose diagonal_a_lift
```

## Gaits

A semantic gait starts from a complete semantic pose. Each phase either references
a semantic pose or applies a sparse `targets` list to the preceding resolved pose.
All targets in a phase interpolate concurrently; no servo movement blocks another.

Compile and inspect a CSV trajectory without hardware access:

```bash
scripts/run_semantic_gait_on_robot.sh examples/semantic/gaits/slow_forward_creep.json
```

Only after validating the profile directions on a supported robot should the same
command be run with `--execute`. The generated trajectory still enforces calibrated
pulse bounds and the selected per-frame pulse-delta limit.

`examples/poses_old` and `examples/gaits_old` retain the previous raw 12-channel
authoring format for comparison and regression tests.
