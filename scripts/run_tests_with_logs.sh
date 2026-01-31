#!/usr/bin/env bash
# Run integration tests and write logs to test_logs/ (visible on host).
# Run from repo root: ./scripts/run_tests_with_logs.sh
set -e
cd "$(dirname "$0")/.."
mkdir -p test_logs
chmod 777 test_logs
# Mount only our packages so vendored src/ros2_robotiq_gripper stays (install symlinks point there)
docker compose run --rm \
  -v "$(pwd)/src/ur10_description:/home/ros/ws/src/ur10_description:ro" \
  -v "$(pwd)/src/ur10_gz_bringup:/home/ros/ws/src/ur10_gz_bringup:ro" \
  ur10_sim bash -c "
  . /opt/ros/jazzy/setup.bash && . /home/ros/ws/install/setup.bash &&
  mkdir -p /home/ros/ws/test_logs &&
  colcon test --packages-select ur10_gz_bringup --event-handlers console_direct+ 2>&1 | tee /home/ros/ws/test_logs/ur10_gz_bringup_test.log &&
  colcon test-result --all &&
  cp -r build/ur10_gz_bringup/Testing build/ur10_gz_bringup/test_results /home/ros/ws/test_logs/ 2>/dev/null || true
"
echo "Logs written to test_logs/ (e.g. test_logs/ur10_gz_bringup_test.log)"
