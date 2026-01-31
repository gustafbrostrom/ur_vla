#!/usr/bin/env bash
# Save one frame from /wrist_camera/image_raw to a file.
# Requires the sim to be running (docker compose up). Writes to ./output/wrist_camera.png by default.
# Usage: ./scripts/save_wrist_image.sh [output_path]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OUTPUT_DIR="$REPO_ROOT/output"
OUTPUT_PATH="${1:-$OUTPUT_DIR/wrist_camera.png}"

mkdir -p "$OUTPUT_DIR"
# Container runs as user ros (UID 1001); allow it to write into output
chmod 777 "$OUTPUT_DIR"
cd "$REPO_ROOT"

docker compose exec ur10_sim bash -c "
  . /opt/ros/jazzy/setup.bash && . /home/ros/ws/install/setup.bash &&
  python3 /home/ros/ws/scripts/save_wrist_image.py /home/ros/ws/output/wrist_camera.png
"

if [ "$OUTPUT_PATH" != "$OUTPUT_DIR/wrist_camera.png" ]; then
  cp "$OUTPUT_DIR/wrist_camera.png" "$OUTPUT_PATH"
  echo "Copied to $OUTPUT_PATH"
fi
