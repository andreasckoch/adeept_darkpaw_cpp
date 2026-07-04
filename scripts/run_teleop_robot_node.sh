#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

build_dir="${repo_root}/build"
profile="${repo_root}/examples/semantic/darkpaw_profile.json"
poses_dir="${repo_root}/examples/semantic/poses"
bind_address="0.0.0.0"
port="45454"
execute=0
extra_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir)
      build_dir="$2"
      shift 2
      ;;
    --profile)
      profile="$2"
      shift 2
      ;;
    --poses-dir)
      poses_dir="$2"
      shift 2
      ;;
    --bind)
      bind_address="$2"
      shift 2
      ;;
    --port)
      port="$2"
      shift 2
      ;;
    --execute)
      execute=1
      shift
      ;;
    --i2c-bus|--address)
      extra_args+=("$1" "$2")
      shift 2
      ;;
    --help|-h)
      cat <<USAGE
Usage: $0 [--build-dir DIR] [--profile FILE] [--poses-dir DIR] [--bind A.B.C.D] [--port N] [--execute]

Dry-run is the default. Add --execute only when the robot is supported, powered safely, and the operator is ready.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

binary="${build_dir}/spider_teleop_robot_node"
if [[ ! -x "${binary}" ]]; then
  echo "Missing executable: ${binary}" >&2
  echo "Build the project first, for example: cmake --build ${build_dir}" >&2
  exit 1
fi

args=(--profile "${profile}" --poses-dir "${poses_dir}" --bind "${bind_address}" --port "${port}")
if [[ "${execute}" -eq 1 ]]; then
  args+=(--execute)
fi
args+=("${extra_args[@]}")

exec "${binary}" "${args[@]}"
