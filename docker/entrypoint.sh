#!/bin/bash
set -e
# Source ROS 2 and workspace
. /opt/ros/jazzy/setup.bash
if [ -f /home/ros/ws/install/setup.bash ]; then
  . /home/ros/ws/install/setup.bash
fi
# Forward DISPLAY for X11 GUI
export DISPLAY="${DISPLAY:-:0}"
exec "$@"
