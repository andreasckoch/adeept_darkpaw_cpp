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
mkdir -p build
cd build
cmake ..
make
```

## Hardware diagnostics

Before running any motion code, run the read-only diagnostics executable:

```bash
sudo ./spider_diagnostics
```

Useful options:

```bash
sudo ./spider_diagnostics --i2c-bus 1 --address 0x40
sudo ./spider_diagnostics --skip-camera
```

The diagnostics command checks pigpio initialization, available I2C devices, read-only PCA9685 registers, camera detection tooling, and prints a servo power checklist. It does not command servos.

## Gait dry run

Use the dry-run gait command to inspect bounded high-level velocity intent without opening pigpio, I2C, or commanding servos:

```bash
./spider_gait_dry_run --vx 0.05 --vy 0.0 --yaw 0.2
```

The command prints the raw command, bounded command, selected gait mode, and 12 intended servo pulse/tick targets.

## Hand-designed gait authoring

Hand-designed gaits are authored as JSON poses and JSON gait definitions, then compiled into CSV trajectories. These tools are hardware-free: they do not read actual servo position, open pigpio, use I2C, or command servos.

Validate the example neutral hold gait:

```bash
./spider_validate_gait --gait examples/gaits/hold_neutral.json --poses examples/poses
```

Compile it to a timestamped pulse/tick trajectory:

```bash
./spider_compile_gait \
  --gait examples/gaits/hold_neutral.json \
  --poses examples/poses \
  --output build/hold_neutral.csv
```

Replay the compiled CSV as text:

```bash
./spider_replay_gait --trajectory build/hold_neutral.csv
```

Pose files live in `examples/poses` and contain exactly 12 servo channels. Short excerpt:

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

- `pca9685`: low-level pigpio/I2C access, PCA9685 register writes, PWM frequency setup, and pulse-width conversion helpers.
- `servo_calibration`: per-servo pulse limits and clamping.
- `motion_command`: high-level velocity/yaw command validation, deadzone handling, and clamping.
- `gait_controller`: hardware-free dry-run gait target generation from bounded commands.
- `gait_pose`, `gait_definition`, `gait_trajectory`: hardware-free JSON pose/gait authoring, validation, CSV trajectory compilation, and replay support.
- `servo`: current gait prototype and safe servo pulse API built on top of the PCA9685 HAL and calibration layer.
