#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

build_dir="${repo_root}/build"
host="127.0.0.1"
port="45454"
rate_hz="30"
hold_ms="180"
speed="0.50"
dry_run=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir)
      build_dir="$2"
      shift 2
      ;;
    --host)
      host="$2"
      shift 2
      ;;
    --port)
      port="$2"
      shift 2
      ;;
    --rate-hz)
      rate_hz="$2"
      shift 2
      ;;
    --hold-ms)
      hold_ms="$2"
      shift 2
      ;;
    --speed)
      speed="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    --help|-h)
      cat <<USAGE
Usage: $0 --host ROBOT_IP [--build-dir DIR] [--port N] [--rate-hz N] [--hold-ms N] [--speed SCALE] [--dry-run]

Keys: W/A/S/D move, Q/E rotate, space stops, X sends estop, Ctrl-C exits.
USAGE
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

binary="${build_dir}/spider_teleop_keyboard"
if [[ ! -x "${binary}" ]]; then
  echo "Missing executable: ${binary}" >&2
  echo "Build the project first, for example: cmake --build ${build_dir}" >&2
  exit 1
fi

args=(--host "${host}" --port "${port}" --rate-hz "${rate_hz}" --hold-ms "${hold_ms}" --speed "${speed}")
if [[ "${dry_run}" -eq 1 ]]; then
  args+=(--dry-run)
fi

exec "${binary}" "${args[@]}"
