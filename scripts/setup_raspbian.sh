#!/usr/bin/env bash
set -euo pipefail

# Bootstrap a Raspberry Pi OS 12 (Bookworm) image for the Adeept Darkpaw C++
# project. Run this on the Raspberry Pi, not on the Mac.
#
# Usage:
#   chmod +x scripts/setup_raspbian.sh
#   ./scripts/setup_raspbian.sh
#
# The script installs build tools, Raspberry Pi hardware libraries, I2C tools,
# camera/streaming diagnostics, and lightweight development utilities. It does
# not run the robot binary or command servos.

if [[ "$(uname -m)" != "aarch64" && "$(uname -m)" != "armv7l" ]]; then
    echo "Warning: this does not look like a Raspberry Pi OS ARM system."
    echo "Continuing anyway; package availability may differ."
fi

if [[ $EUID -eq 0 ]]; then
    SUDO=""
    TARGET_USER="${SUDO_USER:-root}"
else
    SUDO="sudo"
    TARGET_USER="$(id -un)"
fi

if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    echo "Detected OS: ${PRETTY_NAME:-unknown}"
    if [[ "${VERSION_CODENAME:-}" != "bookworm" ]]; then
        echo "Warning: expected Raspberry Pi OS/Debian bookworm; detected '${VERSION_CODENAME:-unknown}'."
    fi
fi

install_if_available() {
    local pkg="$1"
    if apt-cache show "$pkg" >/dev/null 2>&1; then
        echo "Installing ${pkg}"
        $SUDO apt-get install -y "$pkg"
    else
        echo "Skipping ${pkg}: package not available in configured apt sources"
    fi
}

install_group() {
    local label="$1"
    shift

    echo
    echo "==> ${label}"
    for pkg in "$@"; do
        install_if_available "$pkg"
    done
}

echo "==> Updating package indexes"
$SUDO apt-get update

install_group "Core build and repo tools" \
    build-essential \
    cmake \
    git \
    pkg-config \
    gdb \
    clang-format \
    cppcheck

install_group "Raspberry Pi hardware and I2C support" \
    raspi-config \
    i2c-tools \
    pigpio \
    libpigpio-dev \
    libraspberrypi-dev \
    raspberrypi-kernel-headers

install_group "Python helpers for diagnostics and future tooling" \
    python3 \
    python3-pip \
    python3-venv \
    python3-smbus \
    python3-yaml \
    python3-numpy

install_group "Camera and low-latency streaming diagnostics" \
    rpicam-apps \
    libcamera-apps \
    libcamera-tools \
    v4l-utils \
    ffmpeg \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libcamera \
    gstreamer1.0-libav

install_group "Network and system diagnostics" \
    openssh-server \
    avahi-daemon \
    net-tools \
    iproute2 \
    lsof \
    htop \
    tmux \
    minicom

echo
echo "==> Enabling Raspberry Pi interfaces"
if command -v raspi-config >/dev/null 2>&1; then
    $SUDO raspi-config nonint do_i2c 0 || echo "Warning: failed to enable I2C via raspi-config"
    $SUDO raspi-config nonint do_ssh 0 || echo "Warning: failed to enable SSH via raspi-config"
    $SUDO raspi-config nonint do_camera 0 || echo "Warning: failed to enable camera via raspi-config"
else
    echo "raspi-config is unavailable; enable I2C, SSH, and camera manually."
fi

echo
echo "==> Enabling useful services"
if command -v systemctl >/dev/null 2>&1; then
    $SUDO systemctl enable --now ssh || echo "Warning: failed to enable ssh service"
    $SUDO systemctl enable --now avahi-daemon || echo "Warning: failed to enable avahi-daemon service"

    # The current C++ code uses the direct pigpio C API through gpioInitialise().
    # It should normally be run with sudo and does not need the pigpiod daemon.
    # Keeping pigpiod disabled avoids contention during direct hardware access.
    $SUDO systemctl disable --now pigpiod >/dev/null 2>&1 || true
fi

echo
echo "==> Adding ${TARGET_USER} to hardware-access groups"
for group in gpio i2c spi video input dialout; do
    if getent group "$group" >/dev/null 2>&1; then
        $SUDO usermod -aG "$group" "$TARGET_USER"
    fi
done

echo
echo "==> Setup complete"
echo "Reboot before hardware testing so interface and group changes apply:"
echo "  sudo reboot"
echo
echo "After reboot, useful checks are:"
echo "  i2cdetect -y 1              # PCA9685 should usually appear at 0x40"
echo "  rpicam-hello --list-cameras # camera detection on Bookworm"
echo "  groups                      # confirm gpio/i2c/video/input membership"
echo
echo "Build check from the repo root:"
echo "  mkdir -p build && cd build && cmake .. && make"
echo
echo "Do not run the robot binary until the robot is physically safe to move."
