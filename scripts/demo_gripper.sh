#!/usr/bin/env bash
# Demo: open and close the Robotiq 2F-85 gripper via GripperCommand action.
# Run with sim up; optionally pass "open" or "close" (default: open then close).
# Usage: ./scripts/demo_gripper.sh [open|close|both]
set -e

# Source ROS 2 and workspace if available
if [ -f /opt/ros/jazzy/setup.bash ]; then
  . /opt/ros/jazzy/setup.bash
fi
if [ -f /home/ros/ws/install/setup.bash ]; then
  . /home/ros/ws/install/setup.bash
fi

ACTION="${1:-both}"

open_gripper() {
  echo "Opening gripper (position 0.0 rad)..."
  ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 0.0}}"
}

close_gripper() {
  echo "Closing gripper (position 0.7929 rad)..."
  ros2 action send_goal /gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.7929, max_effort: 0.0}}"
}

case "$ACTION" in
  open)  open_gripper ;;
  close) close_gripper ;;
  both)  open_gripper; close_gripper ;;
  *)     echo "Usage: $0 [open|close|both]"; exit 1 ;;
esac

echo "Done. To check joint state: ros2 topic echo /joint_states --once | grep -A1 robotiq_85_left_knuckle_joint"
