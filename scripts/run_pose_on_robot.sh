#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: scripts/run_pose_on_robot.sh --pose POSE.json [options]

Options:
  --pose POSE         Pose file path, pose filename, or pose name without .json.
  --execute           Actually command the PCA9685 via spider_play_gait.
  --poses DIR         Legacy raw pose directory. Default: examples/poses_old, or the pose file directory.
  --output-dir DIR    Compiled CSV directory. Default: data/gaits
  --build-dir DIR     Build directory. Default: build
  --speed SCALE       Playback speed scale, 0 < SCALE <= 1. Default: 0.10
  --max-delta-us N    Max allowed per-frame pulse delta. Default: 80
  --duration-ms N     Duration for each neutral-to-pose and pose-to-neutral phase. Default: 1600
  --steps N           Interpolation steps per phase. Default: 16
  --i2c-bus N         PCA9685 I2C bus. Default: 1
  --address ADDR      PCA9685 address. Default: 0x40

This script expands a pose into neutral_stand -> POSE -> neutral_stand.
Without --execute, it compiles, validates, and dry-runs only.
USAGE
}

require_positive_int() {
  local name="$1"
  local value="$2"
  if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
    echo "${name} must be a positive integer: ${value}" >&2
    exit 2
  fi
}

pose_arg=""
poses_dir="examples/poses_old"
poses_dir_explicit="0"
output_dir="data/gaits"
build_dir="build"
speed="0.10"
max_delta_us="80"
duration_ms="1600"
steps="16"
i2c_bus="1"
address="0x40"
execute="0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --pose)
      pose_arg="$2"
      shift 2
      ;;
    --execute)
      execute="1"
      shift
      ;;
    --poses)
      poses_dir="$2"
      poses_dir_explicit="1"
      shift 2
      ;;
    --output-dir)
      output_dir="$2"
      shift 2
      ;;
    --build-dir)
      build_dir="$2"
      shift 2
      ;;
    --speed)
      speed="$2"
      shift 2
      ;;
    --max-delta-us)
      max_delta_us="$2"
      shift 2
      ;;
    --duration-ms)
      duration_ms="$2"
      shift 2
      ;;
    --steps)
      steps="$2"
      shift 2
      ;;
    --i2c-bus)
      i2c_bus="$2"
      shift 2
      ;;
    --address)
      address="$2"
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

if [[ -z "$pose_arg" ]]; then
  usage
  exit 2
fi

pose_file="$pose_arg"
if [[ ! -f "$pose_file" ]]; then
  pose_name_without_ext="${pose_arg%.json}"
  pose_file="${poses_dir}/${pose_name_without_ext}.json"
fi

if [[ ! -f "$pose_file" ]]; then
  echo "Pose file not found: ${pose_arg}" >&2
  exit 1
fi

if [[ "$poses_dir_explicit" == "0" && "$pose_file" == */* ]]; then
  poses_dir="$(dirname "$pose_file")"
fi

pose_file_name="$(basename "$pose_file")"
pose_name="${pose_file_name%.json}"
if [[ ! "$pose_name" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "Pose filename must contain only letters, numbers, underscore, or dash: ${pose_file_name}" >&2
  exit 1
fi

require_positive_int "--max-delta-us" "$max_delta_us"
require_positive_int "--duration-ms" "$duration_ms"
require_positive_int "--steps" "$steps"

if [[ ! -f "${poses_dir}/neutral_stand.json" ]]; then
  echo "Missing neutral pose: ${poses_dir}/neutral_stand.json" >&2
  exit 1
fi

for tool in spider_validate_gait spider_compile_gait spider_replay_gait spider_play_gait; do
  if [[ ! -x "${build_dir}/${tool}" ]]; then
    echo "Missing ${build_dir}/${tool}; build the project first." >&2
    exit 1
  fi
done

mkdir -p "$output_dir"
trajectory_path="${output_dir}/pose_${pose_name}.csv"
gait_path="$(mktemp "${TMPDIR:-/tmp}/spider_pose_gait.XXXXXX.json")"
trap 'rm -f "$gait_path"' EXIT

cat > "$gait_path" <<GAIT_JSON
{
  "name": "pose_${pose_name}",
  "description": "Generated pose check gait for ${pose_name}.",
  "phases": [
    { "name": "move_to_${pose_name}", "from": "neutral_stand", "to": "${pose_name}", "duration_ms": ${duration_ms}, "steps": ${steps} },
    { "name": "return_neutral", "from": "${pose_name}", "to": "neutral_stand", "duration_ms": ${duration_ms}, "steps": ${steps} }
  ]
}
GAIT_JSON

"${build_dir}/spider_validate_gait" \
  --gait "$gait_path" \
  --poses "$poses_dir" \
  --max-delta-us "$max_delta_us"

"${build_dir}/spider_compile_gait" \
  --gait "$gait_path" \
  --poses "$poses_dir" \
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
