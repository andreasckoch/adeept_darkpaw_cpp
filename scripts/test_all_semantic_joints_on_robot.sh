#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: scripts/test_all_semantic_joints_on_robot.sh [options]

Tests all 12 standard Darkpaw semantic joints serially. Each joint visits both
profile endpoints and returns to profile neutral before the next joint begins.

Options:
  --profile FILE      Semantic robot profile. Default: examples/semantic/darkpaw_profile.json
  --cycles N          Complete endpoint cycles per joint. Default: 1
  --step-us N         Pulse increment for each move. Default: 100
  --step-delay-ms N   Delay after each increment. Default: 20
  --settle-ms N       Delay at each target. Default: 250
  --build-dir DIR     Build directory. Default: build
  --i2c-bus N         PCA9685 I2C bus. Default: 1
  --address ADDR      PCA9685 address. Default: 0x40
  --execute           Actually command the joints.

The defaults are faster than the single-joint tester while still commanding both
semantic endpoint values. Without --execute, this is a dry run.
USAGE
}

require_option_value() {
  if [[ $# -lt 2 ]]; then
    echo "Missing value for $1" >&2
    exit 2
  fi
}

profile="examples/semantic/darkpaw_profile.json"
cycles="1"
step_us="100"
step_delay_ms="20"
settle_ms="250"
build_dir="build"
i2c_bus="1"
address="0x40"
execute="0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile|--cycles|--step-us|--step-delay-ms|--settle-ms|--build-dir|--i2c-bus|--address)
      require_option_value "$@"
      case "$1" in
        --profile) profile="$2" ;;
        --cycles) cycles="$2" ;;
        --step-us) step_us="$2" ;;
        --step-delay-ms) step_delay_ms="$2" ;;
        --settle-ms) settle_ms="$2" ;;
        --build-dir) build_dir="$2" ;;
        --i2c-bus) i2c_bus="$2" ;;
        --address) address="$2" ;;
      esac
      shift 2
      ;;
    --execute)
      execute="1"
      shift
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

tool_path="${build_dir}/spider_test_semantic_joint"
if [[ ! -x "$tool_path" ]]; then
  echo "Missing ${tool_path}; build the project first." >&2
  exit 1
fi

legs=(front_left rear_left front_right rear_right)
axes=(fore_aft lift stance)
for leg in "${legs[@]}"; do
  for axis in "${axes[@]}"; do
    echo "== Testing ${leg}.${axis} =="
    args=(
      --profile "$profile"
      --leg "$leg"
      --axis "$axis"
      --cycles "$cycles"
      --step-us "$step_us"
      --step-delay-ms "$step_delay_ms"
      --settle-ms "$settle_ms"
      --i2c-bus "$i2c_bus"
      --address "$address"
    )
    if [[ "$execute" == "1" ]]; then
      sudo "$tool_path" "${args[@]}" --execute
    else
      "$tool_path" "${args[@]}"
    fi
  done
done
