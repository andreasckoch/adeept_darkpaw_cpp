# Adeept Darkpaw C++
This is a hobby project for controlling the movement of the Adeept Darkpaw spider robot. 
It uses a Robot HAT extension for the Rasberry Pi and has 12 servo motors to be controlled via the I2C protocol.

## Dependencies
- CMake
- C++ compiler
- pigpio

## Raspberry Pi setup

On a fresh Raspberry Pi OS 12/Bookworm image, run:

```bash
scripts/setup_raspbian.sh
sudo reboot
```

After reboot, check that the PCA9685 is visible on the expected I2C bus:

```bash
i2cdetect -y 1
```

The PCA9685 should usually appear at address `0x40`.

## Build

```bash
cmake -S . -B build
cmake --build build
```

## Hardware diagnostics

Before running any motion code, run the read-only diagnostics executable:

```bash
sudo ./build/spider_diagnostics
```

Useful options:

```bash
sudo ./build/spider_diagnostics --i2c-bus 1 --address 0x40
sudo ./build/spider_diagnostics --skip-camera
```

The diagnostics command checks pigpio initialization, available I2C devices, read-only PCA9685 registers, camera detection tooling, and prints a servo power checklist. It does not command servos.

## Gait dry run

Use the dry-run gait command to inspect bounded high-level velocity intent without opening pigpio, I2C, or commanding servos:

```bash
./build/spider_gait_dry_run --vx 0.05 --vy 0.0 --yaw 0.2
```

The command prints the raw command, bounded command, selected gait mode, and 12 intended servo pulse/tick targets.

## Hand-designed gait authoring

Hand-designed gaits are authored as JSON poses and JSON gait definitions, then compiled into CSV trajectories. Validation, compilation, and replay are hardware-free: they do not read actual servo position, open pigpio, use I2C, or command servos.

Validate the example neutral hold gait:

```bash
./build/spider_validate_gait \
  --gait examples/gaits/hold_neutral.json \
  --poses examples/poses
```

Compile it to the dedicated generated-gait directory:

```bash
mkdir -p data/gaits
./build/spider_compile_gait \
  --gait examples/gaits/slow_forward_creep.json \
  --poses examples/poses \
  --output data/gaits/slow_forward_creep.csv \
  --max-delta-us 80
```

Replay the compiled CSV as text:

```bash
./build/spider_replay_gait --trajectory data/gaits/slow_forward_creep.csv --max-delta-us 80
```

The hardware player is dry-run by default:

```bash
./build/spider_play_gait \
  --trajectory data/gaits/slow_forward_creep.csv \
  --speed 0.10 \
  --max-delta-us 80
```

To intentionally move the robot on the Raspberry Pi, keep the robot lifted or supported, keep power reachable, and add `--execute`:

```bash
sudo ./build/spider_play_gait \
  --trajectory data/gaits/slow_forward_creep.csv \
  --speed 0.10 \
  --max-delta-us 80 \
  --execute
```

The wrapper script performs the validate, compile, text replay, and optional hardware play sequence:

```bash
scripts/run_gait_on_robot.sh examples/gaits/slow_forward_creep.json
scripts/run_gait_on_robot.sh examples/gaits/slow_forward_creep.json --execute
```

To test a single pose, use the pose runner. It generates a temporary `neutral_stand -> <pose> -> neutral_stand` gait and compiles the CSV into `data/gaits`:

```bash
scripts/run_pose_on_robot.sh --pose examples/poses/diagonal_a_lift.json
scripts/run_pose_on_robot.sh --pose diagonal_a_lift --execute
```

To diagnose one disconnected or unloaded actuator, use the single-channel test. It
defaults to channel 5 and writes only the selected PCA9685 channel. It starts with
a conservative excursion covering 25% of the calibrated range on either side of
centre; increase `--range-percent` only after verifying the horn and linkage are
clear. Dry-run first, then explicitly execute on the Pi:

```bash
scripts/test_servo_on_robot.sh
scripts/test_servo_on_robot.sh --execute
```

The test moves centre -> low -> centre -> high -> centre in 25 us increments. The
servo remains commanded at centre after completion, so turn off servo power before
changing wiring or removing the horn.

Pose files live in `examples/poses` and contain exactly 12 servo channels. The seeded Darkpaw poses are conservative unvalidated starting points based on the repository calibration limits and public 12-servo Darkpaw context; they are not measured kinematic solutions. Short excerpt:

```json
{
  "name": "neutral_stand",
  "description": "Conservative neutral standing pose.",
  "servos": [
    { "channel": 0, "pulse_microsec": 1040 }
  ]
}
```

Gait files live in `examples/gaits` and reference pose names. A phase interpolates from one pose to another over `duration_ms` and `steps`:

```json
{
  "name": "hold_neutral",
  "description": "Hold the neutral pose.",
  "phases": [
    {
      "name": "hold",
      "from": "neutral_stand",
      "to": "neutral_stand",
      "duration_ms": 1000,
      "steps": 10
    }
  ]
}
```

The compiler output is CSV:

```csv
timestamp_ms,phase,step,channel,pulse_microsec,ticks
0,hold,0,0,1040,212
```

## Tests

Pure conversion, calibration, command-bounding, dry-run gait, and gait authoring tests do not require robot hardware:

```bash
ctest --test-dir build --output-on-failure
```

## Code structure

- `src/hal`: low-level pigpio/I2C access, PCA9685 register writes, PWM frequency setup, and pulse-width conversion helpers.
- `src/actuation`: per-servo pulse limits, clamping, and the current servo pulse API built on top of the PCA9685 HAL.
- `src/gait`: hardware-free motion commands, dry-run gait target generation, JSON pose/gait authoring, validation, CSV trajectory compilation, and replay support.
- `src/tools`: executable entry points for diagnostics, dry-run gait, gait authoring, playback, and the current spider robot prototype.
- `src/common`: shared small utilities.
