#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
if [ -f /catkin_ws/devel/setup.bash ]; then
  source /catkin_ws/devel/setup.bash
fi

if [ "$(uname -m)" = "x86_64" ] && [ -d /catkin_ws/src/seeker1/libs/x86 ]; then
  export LD_LIBRARY_PATH="/catkin_ws/src/seeker1/libs/x86${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
elif [ -d /catkin_ws/src/seeker1/libs/arm64 ]; then
  export LD_LIBRARY_PATH="/catkin_ws/src/seeker1/libs/arm64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

exec "$@"
