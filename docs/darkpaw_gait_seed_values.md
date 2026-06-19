# Darkpaw Gait Seed Values

The example pose and gait files under `examples/poses` and `examples/gaits` are conservative seed data for slow manual testing. They are not a measured Adeept Darkpaw kinematic model.

Public Adeept Darkpaw material is useful for confirming the overall platform context: Raspberry Pi robot, Robot HAT/PCA9685-style servo control, and 12 servo channels. It does not provide a complete, trustworthy channel-to-joint map, link geometry, neutral pose, or calibrated gait table for this C++ repository.

For that reason, the seeded values are derived from:

- The repository's current per-servo pulse limits in `src/actuation/servo_calibration.cpp`.
- A neutral pose near the center of those limits.
- Small pulse offsets that keep every frame inside calibration bounds and below the default playback delta limit.
- Diagonal-pair phase patterns intended to support slow lifted tests before floor contact tests.

Before treating any seeded gait as real walking behavior:

1. Lift or support the robot so legs can move without loading the frame.
2. Run `scripts/run_gait_on_robot.sh examples/gaits/hold_neutral.json --execute`.
3. Use `scripts/run_pose_on_robot.sh --pose <pose-name> --execute` for slow isolated pose checks.
4. Confirm every channel maps to the expected physical joint.
5. Record corrected neutral and lift/advance/retract poses as new JSON files.
6. Recompile into `data/gaits` and replay at low speed before testing on the floor.

If a channel moves the wrong joint or direction, update the pose JSON rather than compensating in the player. The player should remain a simple, bounded trajectory executor.
