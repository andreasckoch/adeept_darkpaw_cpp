#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: scripts/test_semantic_joint_on_robot.sh --leg NAME --axis AXIS [options]

Tests all three profile values for one semantic joint and returns to neutral.

Options:
  --leg NAME          Leg name from the profile, for example front_left.
  --axis AXIS         fore_aft, lift, or stance.
  --profile FILE      Semantic robot profile. Default: examples/semantic/darkpaw_profile.json
  --cycles N          Complete endpoint cycles. Default: 1
  --step-us N         Pulse increment for each move. Default: 25
  --step-delay-ms N   Delay after each increment. Default: 100
  --settle-ms N       Delay at each target. Default: 800
  --build-dir DIR     Build directory. Default: build
  --i2c-bus N         PCA9685 I2C bus. Default: 1
  --address ADDR      PCA9685 address. Default: 0x40
  --execute           Actually command the semantic joint.

Without --execute, this is a dry run and does not open hardware.
USAGE
}

require_option_value() {
  if [[ $# -lt 2 ]]; then
    echo "Missing value for $1" >&2
    exit 2
  fi
}

profile="examples/semantic/darkpaw_profile.json"
build_dir="build"
execute="0"
leg=""
axis=""
tool_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --leg|--axis|--profile|--cycles|--step-us|--step-delay-ms|--settle-ms|--i2c-bus|--address|--build-dir)
      require_option_value "$@"
      case "$1" in
        --leg) leg="$2" ;;
        --axis) axis="$2" ;;
        --profile) profile="$2" ;;
        --build-dir) build_dir="$2" ;;
        *) tool_args+=("$1" "$2") ;;
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

if [[ -z "$leg" || -z "$axis" ]]; then
  usage
  exit 2
fi

tool_path="${build_dir}/spider_test_semantic_joint"
if [[ ! -x "$tool_path" ]]; then
  echo "Missing ${tool_path}; build the project first." >&2
  exit 1
fi

base_args=(--profile "$profile" --leg "$leg" --axis "$axis")
if [[ ${#tool_args[@]} -gt 0 ]]; then
  base_args+=("${tool_args[@]}")
fi
if [[ "$execute" == "1" ]]; then
  echo "Executing one semantic joint test. Keep the horn/linkage clear and servo power reachable."
  sudo "$tool_path" "${base_args[@]}" --execute
else
  "$tool_path" "${base_args[@]}"
fi
