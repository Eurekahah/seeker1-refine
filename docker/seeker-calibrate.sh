#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH="/catkin_ws/src/seeker1/libs/x86${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

python3 /catkin_ws/src/seeker1/script/1get_kalibr_info.py
install -m 0644 /tmp/kalibr_cam_chain.yaml /catkin_ws/src/seeker1/config/seeker_omni_depth/kalibr_cam_chain.yaml
echo "Calibration written to /catkin_ws/src/seeker1/config/seeker_omni_depth/kalibr_cam_chain.yaml"
