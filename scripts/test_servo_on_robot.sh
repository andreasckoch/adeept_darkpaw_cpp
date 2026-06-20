#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: scripts/test_servo_on_robot.sh [options]

Tests one PCA9685 servo channel. Channel 5 is the default.

Options:
  --channel N          Servo/PCA9685 channel. Default: 5
  --range-percent N    Excursion on either side of centre, 1-50. Default: 25
  --cycles N           Low-centre-high-centre cycles. Default: 1
  --step-us N          Pulse increment for each move. Default: 25
  --step-delay-ms N    Delay after each increment. Default: 100
  --settle-ms N        Delay at each target. Default: 800
  --build-dir DIR      Build directory. Default: build
  --i2c-bus N          PCA9685 I2C bus. Default: 1
  --address ADDR       PCA9685 address. Default: 0x40
  --execute            Actually command the actuator.

Without --execute, this is a dry run and does not open hardware.
USAGE
}

build_dir="build"
execute="0"
tool_args=()

require_option_value() {
  if [[ $# -lt 2 ]]; then
    echo "Missing value for $1" >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir)
      require_option_value "$@"
      build_dir="$2"
      shift 2
      ;;
    --execute)
      execute="1"
      tool_args+=("$1")
      shift
      ;;
    --channel|--range-percent|--cycles|--step-us|--step-delay-ms|--settle-ms|--i2c-bus|--address)
      require_option_value "$@"
      tool_args+=("$1" "$2")
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      usage
      exit 2
      ;;
  esac
done

tool_path="${build_dir}/spider_test_servo"
if [[ ! -x "$tool_path" ]]; then
  echo "Missing ${tool_path}; build the project first." >&2
  exit 1
fi

if [[ "$execute" == "1" ]]; then
  echo "Executing a single-servo test. Keep the horn/linkage clear and servo power reachable."
  sudo "$tool_path" "${tool_args[@]}"
else
  "$tool_path" "${tool_args[@]}"
fi
