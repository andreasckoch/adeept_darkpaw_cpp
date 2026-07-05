#!/usr/bin/env bash
set -euo pipefail

host=""
port="5000"
width="1280"
height="720"
fps="30"
bitrate="4000000"

usage() {
  cat <<USAGE
Usage: $0 --host MAC_IP [--port N] [--width N] [--height N] [--fps N] [--bitrate N]

Streams Raspberry Pi camera H.264 over UDP using rpicam-vid or libcamera-vid.
The desktop telemetry viewer does not decode this raw UDP stream directly yet;
use VLC/GStreamer/WebRTC as the video display bridge for now.
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      host="$2"
      shift 2
      ;;
    --port)
      port="$2"
      shift 2
      ;;
    --width)
      width="$2"
      shift 2
      ;;
    --height)
      height="$2"
      shift 2
      ;;
    --fps)
      fps="$2"
      shift 2
      ;;
    --bitrate)
      bitrate="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${host}" ]]; then
  usage >&2
  exit 2
fi

if command -v rpicam-vid >/dev/null 2>&1; then
  camera_cmd=(rpicam-vid)
elif command -v libcamera-vid >/dev/null 2>&1; then
  camera_cmd=(libcamera-vid)
else
  echo "Missing rpicam-vid/libcamera-vid. Install rpicam-apps or libcamera-apps." >&2
  exit 1
fi

exec "${camera_cmd[@]}" \
  --timeout 0 \
  --width "${width}" \
  --height "${height}" \
  --framerate "${fps}" \
  --bitrate "${bitrate}" \
  --codec h264 \
  --inline \
  --output "udp://${host}:${port}"
