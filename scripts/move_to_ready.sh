#!/usr/bin/env bash
# Move the UR10 arm to a ready pose (arm forward, elbow bent, wrist level).
# Requires the sim to be running (docker compose up). Run from repo root.
# Usage: ./scripts/move_to_ready.sh [duration_sec]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DURATION_SEC="${1:-3}"

cd "$REPO_ROOT"

docker compose exec ur10_sim bash -c "
  . /opt/ros/jazzy/setup.bash && . /home/ros/ws/install/setup.bash &&
  echo 'Moving arm to ready position (${DURATION_SEC}s)...' &&
  ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    '{
      trajectory: {
        joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
        points: [
          { positions: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0], time_from_start: { sec: ${DURATION_SEC}, nanosec: 0 } }
        ]
      }
    }' &&
  echo 'Done.'
"
