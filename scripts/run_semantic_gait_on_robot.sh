#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: scripts/run_semantic_gait_on_robot.sh GAIT.json [options]

Options:
  --profile FILE      Semantic robot profile. Default: examples/semantic/darkpaw_profile.json
  --poses DIR         Semantic pose directory. Default: examples/semantic/poses
  --output-dir DIR    Compiled CSV directory. Default: data/gaits
  --build-dir DIR     Build directory. Default: build
  --speed SCALE       Playback speed scale, 0 < SCALE <= 1. Default: 0.10
  --max-delta-us N    Max allowed per-frame pulse delta. Default: 80
  --i2c-bus N         PCA9685 I2C bus. Default: 1
  --address ADDR      PCA9685 address. Default: 0x40
  --execute           Actually command the PCA9685 via spider_play_gait.

Without --execute, this script validates, compiles, and dry-runs only.
USAGE
}

require_option_value() {
  if [[ $# -lt 2 ]]; then
    echo "Missing value for $1" >&2
    exit 2
  fi
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

gait_path="$1"
shift
profile="examples/semantic/darkpaw_profile.json"
poses_dir="examples/semantic/poses"
output_dir="data/gaits"
build_dir="build"
speed="0.10"
max_delta_us="80"
i2c_bus="1"
address="0x40"
execute="0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile|--poses|--output-dir|--build-dir|--speed|--max-delta-us|--i2c-bus|--address)
      require_option_value "$@"
      case "$1" in
        --profile) profile="$2" ;;
        --poses) poses_dir="$2" ;;
        --output-dir) output_dir="$2" ;;
        --build-dir) build_dir="$2" ;;
        --speed) speed="$2" ;;
        --max-delta-us) max_delta_us="$2" ;;
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

if [[ ! -f "$gait_path" || ! -f "$profile" || ! -d "$poses_dir" ]]; then
  echo "Missing semantic gait, profile, or pose directory." >&2
  exit 1
fi

for tool in spider_validate_semantic_gait spider_compile_semantic_gait spider_replay_gait spider_play_gait; do
  if [[ ! -x "${build_dir}/${tool}" ]]; then
    echo "Missing ${build_dir}/${tool}; build the project first." >&2
    exit 1
  fi
done

mkdir -p "$output_dir"
gait_file="$(basename "$gait_path")"
gait_name="${gait_file%.json}"
trajectory_path="${output_dir}/${gait_name}.csv"

"${build_dir}/spider_validate_semantic_gait" \
  --profile "$profile" \
  --poses "$poses_dir" \
  --gait "$gait_path" \
  --max-delta-us "$max_delta_us"

"${build_dir}/spider_compile_semantic_gait" \
  --profile "$profile" \
  --poses "$poses_dir" \
  --gait "$gait_path" \
  --output "$trajectory_path" \
  --max-delta-us "$max_delta_us"

"${build_dir}/spider_replay_gait" \
  --trajectory "$trajectory_path" \
  --max-delta-us "$max_delta_us"

if [[ "$execute" == "1" ]]; then
  echo "Executing ${trajectory_path}. Keep the robot lifted/supported and keep power reachable."
  sudo "${build_dir}/spider_play_gait" \
    --trajectory "$trajectory_path" \
    --speed "$speed" \
    --max-delta-us "$max_delta_us" \
    --i2c-bus "$i2c_bus" \
    --address "$address" \
    --execute
else
  "${build_dir}/spider_play_gait" \
    --trajectory "$trajectory_path" \
    --speed "$speed" \
    --max-delta-us "$max_delta_us" \
    --i2c-bus "$i2c_bus" \
    --address "$address"
fi
