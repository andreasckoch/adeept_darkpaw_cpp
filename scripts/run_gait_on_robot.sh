#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: scripts/run_gait_on_robot.sh GAIT.json [options]

Options:
  --execute           Actually command the PCA9685 via spider_play_gait.
  --poses DIR         Legacy raw pose directory. Default: examples/poses_old
  --output-dir DIR    Compiled CSV directory. Default: data/gaits
  --build-dir DIR     Build directory. Default: build
  --speed SCALE       Playback speed scale, 0 < SCALE <= 1. Default: 0.10
  --max-delta-us N    Max allowed per-frame pulse delta. Default: 80
  --i2c-bus N         PCA9685 I2C bus. Default: 1
  --address ADDR      PCA9685 address. Default: 0x40

Without --execute, this script compiles, validates, and dry-runs only.
USAGE
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

gait_path="$1"
shift

poses_dir="examples/poses_old"
output_dir="data/gaits"
build_dir="build"
speed="0.30"
max_delta_us="140"
i2c_bus="1"
address="0x40"
execute="0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute)
      execute="1"
      shift
      ;;
    --poses)
      poses_dir="$2"
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

for tool in spider_validate_gait spider_compile_gait spider_replay_gait spider_play_gait; do
  if [[ ! -x "${build_dir}/${tool}" ]]; then
    echo "Missing ${build_dir}/${tool}; build the project first." >&2
    exit 1
  fi
done

mkdir -p "$output_dir"
gait_file="$(basename "$gait_path")"
gait_name="${gait_file%.*}"
trajectory_path="${output_dir}/${gait_name}.csv"

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
