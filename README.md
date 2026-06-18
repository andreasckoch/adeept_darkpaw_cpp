# Adeept Darkpaw C++
This is a hobby project for controlling the movement of the Adeept Darkpaw spider robot. 
It uses a Robot HAT extension for the Rasberry Pi and has 12 servo motors to be controlled via the I2C protocol.

## Dependencies
- pigpio
- (bcm_host)

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
