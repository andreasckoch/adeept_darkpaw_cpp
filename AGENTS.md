# AGENTS.md

## Project Context

This repository controls an Adeept Darkpaw spider robot based on a Raspberry Pi 4B, a PCA9685 PWM servo controller, a Robot HAT, and 12 servo motors. The current code is a small C++/CMake hardware-control prototype using pigpio and I2C.

The project is expected to grow toward two major applications:

- Steam Deck teleoperation: stream camera data from the robot and control translation with the left joystick and rotation with the right joystick.
- Desktop VLA control: stream camera and actuator state to a desktop PC, run a vision-language-action or robot-learning policy there, and stream bounded actions back to the robot.
- Future depth sensing: evaluate an available Kinect camera as an optional RGB-D sensor for desktop-side perception, mapping, dataset collection, or VLA observations.

Robot hardware can move unexpectedly. Treat hardware execution as a manual, explicit user action.

## Non-Negotiable Safety Rules

- Do not run the robot binary, servo sweep tools, gait demos, or actuator commands automatically from Codex.
- Do not add code that moves servos during initialization unless the user explicitly asks for that behavior.
- Prefer fail-closed control behavior: stale commands, dropped network links, invalid packets, or model uncertainty must stop motion or hold a safe pose.
- Keep an operator-controlled emergency stop path in every architecture that can command actuators.
- Bound all actuator commands by calibrated per-servo limits before they reach the PCA9685.
- Separate "compute decided an action" from "hardware accepted an action"; log both when possible.
- For new motion code, include a dry-run or simulation path before hardware execution.

## Preferred Architecture Direction

Use layered boundaries so teleoperation, ML policies, and low-level servo control can evolve independently.

Recommended long-term layers:

- `hal/`: Raspberry Pi, pigpio, PCA9685, camera, and platform-specific drivers.
- `actuation/`: servo calibration, pulse conversion, inverse kinematics, gait primitives, safety limits.
- `messages/`: stable command, observation, actuator-state, and telemetry schemas.
- `robot_node/`: process running on the Pi that owns hardware access and enforces safety.
- `teleop/`: Steam Deck or gamepad client code.
- `streaming/`: camera/video transport and network transport glue.
- `learning/`: desktop-side dataset logging, policy inference, VLA integration, and replay tools.
- `sim/`: robot model, simulator environments, and synthetic-data or policy-evaluation tasks.
- `tools/`: scripts for build, diagnostics, calibration, log conversion, and dataset export.

Keep the Pi-side runtime small and deterministic. Put expensive model inference on the desktop PC.

## Open-Source Tools To Prefer

For shared robotics middleware:

- ROS 2 is the preferred message/process architecture once the project outgrows the current single binary. Use it for topics such as `cmd_vel`, actuator state, camera metadata, health, and e-stop state.
- `teleop_twist_joy` is a good baseline for gamepad-to-velocity mapping. Adapt its concepts if not adopting ROS 2 immediately.
- `sensor_msgs/Joy`, `geometry_msgs/Twist`, and explicit robot-specific actuator command messages are preferable to ad hoc byte arrays.
- MCAP is the preferred log format for multimodal robot data and replayable datasets.
- Foxglove is a useful visualization/debugging tool for live and recorded robot data.

For camera and low-latency video:

- Raspberry Pi `rpicam-vid`/libcamera tooling is the first baseline for validating camera hardware and network video.
- Treat Kinect support as an optional RGB-D sensor path. Prefer `libfreenect`/`libfreenect2` and ROS 2-compatible wrappers for Kinect experiments, depending on the exact Kinect generation.
- Use Kinect first on the desktop PC if bandwidth, power, USB, or driver constraints make it awkward on the Raspberry Pi.
- GStreamer is the preferred low-level media framework for custom low-latency pipelines.
- WebRTC is a strong candidate when video and bidirectional control need NAT traversal, congestion handling, and browser-like clients.
- UDP H.264/MPEG-TS is acceptable for early LAN-only prototypes, but do not rely on it for reliable control messages.

For Steam Deck teleoperation:

- Treat the Steam Deck as a Linux handheld PC. Prefer a native Linux client or ROS 2 client over a browser-only app if joystick latency and reliable input mapping matter.
- SDL2/SDL3 game controller APIs are good candidates for direct joystick handling.
- Map left stick to planar translation intent and right stick X to yaw/rotation intent. Keep dead zones, scaling, turbo modes, and enable buttons configurable.
- Send operator intent, not raw servo targets, from the Deck. The robot should convert bounded velocity/pose intent into gait and servo commands.

For VLA and robot-learning work:

- LeRobot is the preferred first framework to evaluate for datasets, imitation learning, policies, and async inference because it targets real-world robotics and includes dataset/model tooling.
- OpenVLA is a relevant open-source VLA baseline for desktop-side inference and fine-tuning experiments, but its native action assumptions are manipulation-centric and will need adaptation for a 12-servo legged robot.
- Start VLA integration with logged observations and bounded high-level actions, not direct servo pulses from the model.
- If Kinect is used, log RGB, depth, camera intrinsics, extrinsics, timestamps, and synchronization quality alongside actuator state. Do not assume depth frames are aligned to RGB unless the driver/pipeline explicitly provides aligned output.
- Use policy outputs such as desired body velocity, gait phase parameters, body pose deltas, or footstep targets before considering direct joint/servo commands.
- Keep training/inference code isolated from hardware-control code. The Pi should remain able to reject unsafe actions.

For simulation and training data:

- MuJoCo is a strong first simulator for fast articulated-body dynamics, contact, actuator modeling, and Python-based learning loops.
- Gazebo is a good option when ROS 2 integration, sensors, URDF/SDF models, and robotics tooling matter more than ML throughput.
- Isaac Lab/Isaac Sim is a strong desktop GPU option for large-scale robot-learning experiments, synthetic sensor data, domain randomization, and legged-locomotion-style workflows.
- PyBullet can be used for lightweight experiments, but prefer MuJoCo or Isaac Lab for serious dynamics and policy work unless there is a specific reason.
- Maintain one canonical robot description source or a documented conversion workflow across URDF/SDF/MJCF/USD. Do not let simulator models silently diverge from measured hardware geometry.

## Code Quality Standards

- Prefer small, testable modules over one expanding hardware-control file.
- Use explicit units in names and types where practical: `pulse_microsec`, `counter_ticks`, `angle_rad`, `velocity_mps`, `yaw_rate_radps`.
- Keep conversions centralized and covered by tests, especially microseconds-to-PCA9685 ticks and servo calibration mappings.
- Avoid blocking sleeps inside shared control logic. If timing is needed, isolate it in the runtime layer.
- Validate all external inputs at process boundaries: network packets, joystick values, model actions, config files, and calibration files.
- Do not add new global mutable state unless it is a deliberate hardware singleton with clear lifetime.
- Prefer structured logs for runtime telemetry. Include timestamps, sequence numbers, command age, safety state, and dropped-frame or dropped-command counters.
- Keep Linux/Pi-specific dependencies out of portable logic where reasonable so Mac-side builds and desktop-side tests remain useful.

## Build And Verification Expectations

Before changing existing C++ behavior:

- Inspect the current build system and dependency assumptions.
- Preserve Raspberry Pi compatibility unless the task explicitly migrates the platform.
- Run the smallest relevant local checks available on the Mac.
- If a change needs real Pi hardware, document the exact Pi-side commands for the user to run instead of running them automatically.

For new code, prefer adding one of:

- A pure unit test for conversion, calibration, gait math, message validation, or safety gating.
- A dry-run executable that prints intended commands without touching hardware.
- A simulator or replay test using recorded observations/actions.

## Git And PR Workflow

- Develop every new feature on a dedicated feature branch. Use the `codex/` branch prefix unless the user requests a different branch name.
- Keep commits meaningful and reviewable. Iterative commits are acceptable when they capture coherent progress, working checkpoints, or validated hypotheses.
- Do not mix unrelated cleanup, formatting churn, or dependency changes into feature commits.
- At the end of an implementer flow, push the feature branch and create a pull request when repository access and user permissions allow it.
- If a pull request cannot be created from the current environment, provide the exact commands and PR summary the user should use.

## Stepwise Workflow: Implementer

1. Create or switch to a dedicated feature branch before editing.
2. Read the relevant existing code and identify hardware boundaries before editing.
3. State the intended layer being changed: HAL, actuation, streaming, teleop, learning, simulation, or tooling.
4. Keep the first implementation narrow and observable.
5. Add safety checks before adding convenience behavior.
6. Add or update tests for pure logic.
7. Make meaningful iterative commits when they represent coherent progress.
8. For hardware-affecting changes, provide Pi-side manual verification commands and expected safe output.
9. Run the relevant test suite or explain why it could not be run.
10. Push the branch and create a pull request when possible.
11. Summarize residual risks, especially timing, power, calibration, networking, and model-action risks.

## Stepwise Workflow: Reviewer

1. Do not modify code during reviewer flow unless the user explicitly changes the task from review to implementation.
2. Inspect the diff and form testable hypotheses about safety, correctness, timing, and integration risks.
3. Run the relevant test suite and targeted diagnostic commands available in the current environment.
4. Check whether the change can move hardware unexpectedly.
5. Check whether stale, malformed, or repeated commands fail closed.
6. Verify actuator limits are enforced at the last Pi-side boundary before hardware writes.
7. Verify units and coordinate frames are explicit and consistent.
8. Look for hidden timing assumptions, blocking sleeps, unbounded queues, and missing sequence numbers.
9. Confirm that logs are sufficient to debug a failed robot run.
10. Confirm that pure logic has tests or an executable dry-run path.
11. Report findings first, with file and line references, then summarize test results and residual risks.
12. Call out any missing hardware verification separately from code correctness.

## Suggested Milestones

1. Stabilize current Pi build and document setup for a supported Raspberry Pi OS image.
2. Add hardware diagnostics for pigpio, I2C, PCA9685 detection, camera detection, and servo power assumptions.
3. Refactor servo pulse conversion, calibration limits, and PCA9685 writes behind a small HAL.
4. Add a dry-run gait controller that accepts bounded high-level velocity commands.
5. Add camera streaming baseline with `rpicam-vid`, then evaluate GStreamer/WebRTC for lower-latency integrated control.
6. Evaluate the Kinect as an optional RGB-D sensor path, preferably desktop-attached first, and document driver, power, USB, calibration, and synchronization constraints.
7. Add Steam Deck gamepad input and map it to high-level robot velocity intent.
8. Add MCAP logging for camera metadata, optional Kinect RGB-D metadata, actuator state, command intent, safety state, and timing.
9. Build a simulator model and replay logged commands in simulation.
10. Integrate desktop-side policy inference with a safety-gated high-level action API.
11. Only after replay and simulation are stable, test policy-controlled hardware with reduced speed and an operator e-stop.

## Repository-Specific Notes

- Current CMake links against `pigpio` and `bcm_host`; expect rebuilds on new Raspberry Pi OS images.
- Current servo positions are expressed as pulse widths in microseconds and converted to PCA9685 12-bit counter ticks at 50 Hz.
- The PCA9685 address is currently `0x40` and I2C bus `1` is used.
- A Kinect camera may be incorporated later as an optional RGB-D source; keep Kinect-specific dependencies out of the minimal Pi setup until the exact model and host placement are decided.
- Preserve the existing C++ prototype until a migration path is explicit. If introducing ROS 2 or Python tooling, bridge to the existing hardware layer rather than replacing everything in one change.
