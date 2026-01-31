#!/usr/bin/env bash
# Capture one frame from wrist and external camera, concatenate into one image.
# Requires the sim to be running (docker compose up). Writes to ./output/cameras.png by default.
# Usage: ./scripts/save_camera_images.sh [output_path]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OUTPUT_DIR="$REPO_ROOT/output"
OUTPUT_PATH="${1:-$OUTPUT_DIR/cameras.png}"

mkdir -p "$OUTPUT_DIR"
# Container runs as user ros (UID 1001); allow it to write into output
chmod 777 "$OUTPUT_DIR"
cd "$REPO_ROOT"

docker compose exec ur10_sim bash -c "
  . /opt/ros/jazzy/setup.bash && . /home/ros/ws/install/setup.bash &&
  python3 /home/ros/ws/scripts/save_camera_images.py /home/ros/ws/output/cameras.png
"

if [ "$OUTPUT_PATH" != "$OUTPUT_DIR/cameras.png" ]; then
  cp "$OUTPUT_DIR/cameras.png" "$OUTPUT_PATH"
  echo "Copied to $OUTPUT_PATH"
fi
