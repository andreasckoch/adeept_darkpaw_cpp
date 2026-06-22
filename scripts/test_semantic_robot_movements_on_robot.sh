#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: scripts/test_semantic_robot_movements_on_robot.sh [options]

Builds and runs a coordinated semantic test gait:
  a) all legs: neutral -> up -> neutral -> down -> neutral
  b) stance: neutral -> left -> neutral -> right -> neutral
  c) each leg in turn: raise, neutral -> front -> neutral -> back -> neutral, lower

Options:
  --profile FILE      Semantic robot profile. Default: examples/semantic/darkpaw_profile.json
  --poses DIR         Semantic pose directory. Default: examples/semantic/poses
  --output-dir DIR    Compiled CSV directory. Default: data/gaits
  --build-dir DIR     Build directory. Default: build
  --speed SCALE       Playback speed scale, 0 < SCALE <= 1. Default: 0.25
  --max-delta-us N    Max allowed per-frame pulse delta. Default: 80
  --duration-ms N     Duration for each transition. Default: 800
  --steps N           Interpolation steps per transition. Default: 32
  --i2c-bus N         PCA9685 I2C bus. Default: 1
  --address ADDR      PCA9685 address. Default: 0x40
  --execute           Actually command the robot.

The lateral test uses body-relative stance targets: left legs wide/right legs close,
then the opposite. Without --execute, the generated gait is dry-run only.
USAGE
}

require_option_value() {
  if [[ $# -lt 2 ]]; then
    echo "Missing value for $1" >&2
    exit 2
  fi
}

require_positive_int() {
  local name="$1"
  local value="$2"
  if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
    echo "${name} must be a positive integer: ${value}" >&2
    exit 2
  fi
}

profile="examples/semantic/darkpaw_profile.json"
poses_dir="examples/semantic/poses"
output_dir="data/gaits"
build_dir="build"
speed="0.25"
max_delta_us="80"
duration_ms="800"
steps="32"
i2c_bus="1"
address="0x40"
execute="0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile|--poses|--output-dir|--build-dir|--speed|--max-delta-us|--duration-ms|--steps|--i2c-bus|--address)
      require_option_value "$@"
      case "$1" in
        --profile) profile="$2" ;;
        --poses) poses_dir="$2" ;;
        --output-dir) output_dir="$2" ;;
        --build-dir) build_dir="$2" ;;
        --speed) speed="$2" ;;
        --max-delta-us) max_delta_us="$2" ;;
        --duration-ms) duration_ms="$2" ;;
        --steps) steps="$2" ;;
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

require_positive_int "--max-delta-us" "$max_delta_us"
require_positive_int "--duration-ms" "$duration_ms"
require_positive_int "--steps" "$steps"

if [[ ! -f "$profile" || ! -d "$poses_dir" ]]; then
  echo "Missing semantic profile or pose directory." >&2
  exit 1
fi

for tool in spider_validate_semantic_gait spider_compile_semantic_gait spider_replay_gait spider_play_gait; do
  if [[ ! -x "${build_dir}/${tool}" ]]; then
    echo "Missing ${build_dir}/${tool}; build the project first." >&2
    exit 1
  fi
done

gait_path="$(mktemp "${TMPDIR:-/tmp}/spider_semantic_robot_test.XXXXXX")"
trap 'rm -f "$gait_path"' EXIT
phase_count=0

write_phase() {
  local name="$1"
  local targets="$2"
  if [[ "$phase_count" -gt 0 ]]; then
    printf ',\n'
  fi
  printf '    { "name": "%s", "targets": [%s], "duration_ms": %s, "steps": %s }' \
    "$name" "$targets" "$duration_ms" "$steps"
  phase_count=$((phase_count + 1))
}

{
  cat <<'GAIT_HEADER'
{
  "name": "semantic_robot_movement_test",
  "description": "Generated coordinated semantic profile test. Dry-run by default.",
  "initial_pose": "neutral_stand",
  "phases": [
GAIT_HEADER

  write_phase "all_up" '{ "legs": ["front_left", "rear_left", "front_right", "rear_right"], "axis": "lift", "position": "up" }'
  write_phase "all_lift_neutral" '{ "legs": ["front_left", "rear_left", "front_right", "rear_right"], "axis": "lift", "position": "neutral" }'
  write_phase "all_down" '{ "legs": ["front_left", "rear_left", "front_right", "rear_right"], "axis": "lift", "position": "down" }'
  write_phase "all_lift_neutral_again" '{ "legs": ["front_left", "rear_left", "front_right", "rear_right"], "axis": "lift", "position": "neutral" }'

  write_phase "stance_left" '{ "legs": ["front_left", "rear_left"], "axis": "stance", "position": "wide" }, { "legs": ["front_right", "rear_right"], "axis": "stance", "position": "close" }'
  write_phase "stance_neutral" '{ "legs": ["front_left", "rear_left", "front_right", "rear_right"], "axis": "stance", "position": "neutral" }'
  write_phase "stance_right" '{ "legs": ["front_left", "rear_left"], "axis": "stance", "position": "close" }, { "legs": ["front_right", "rear_right"], "axis": "stance", "position": "wide" }'
  write_phase "stance_neutral_again" '{ "legs": ["front_left", "rear_left", "front_right", "rear_right"], "axis": "stance", "position": "neutral" }'

  for leg in front_left rear_left front_right rear_right; do
    write_phase "${leg}_raise" "{ \"legs\": [\"${leg}\"], \"axis\": \"lift\", \"position\": \"up\" }"
    write_phase "${leg}_front" "{ \"legs\": [\"${leg}\"], \"axis\": \"fore_aft\", \"position\": \"front\" }"
    write_phase "${leg}_fore_aft_neutral" "{ \"legs\": [\"${leg}\"], \"axis\": \"fore_aft\", \"position\": \"neutral\" }"
    write_phase "${leg}_back" "{ \"legs\": [\"${leg}\"], \"axis\": \"fore_aft\", \"position\": \"back\" }"
    write_phase "${leg}_fore_aft_neutral_again" "{ \"legs\": [\"${leg}\"], \"axis\": \"fore_aft\", \"position\": \"neutral\" }"
    write_phase "${leg}_lower" "{ \"legs\": [\"${leg}\"], \"axis\": \"lift\", \"position\": \"neutral\" }"
  done

  cat <<'GAIT_FOOTER'

  ]
}
GAIT_FOOTER
} > "$gait_path"

runner_args=(
  "$gait_path"
  --profile "$profile"
  --poses "$poses_dir"
  --output-dir "$output_dir"
  --build-dir "$build_dir"
  --speed "$speed"
  --max-delta-us "$max_delta_us"
  --i2c-bus "$i2c_bus"
  --address "$address"
)
if [[ "$execute" == "1" ]]; then
  runner_args+=(--execute)
fi

scripts/run_semantic_gait_on_robot.sh "${runner_args[@]}"

generated_trajectory="${output_dir}/$(basename "$gait_path").csv"
final_trajectory="${output_dir}/semantic_robot_movement_test.csv"
if [[ -f "$generated_trajectory" && "$generated_trajectory" != "$final_trajectory" ]]; then
  mv "$generated_trajectory" "$final_trajectory"
fi
echo "Trajectory: ${final_trajectory}"
