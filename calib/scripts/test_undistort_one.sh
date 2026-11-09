#!/bin/bash
source /opt/ros/noetic/setup.bash
pkill -f undistort_one.launch 2>/dev/null || true
sleep 1
nohup roslaunch /data/local/tmp/undistort_one.launch config:=/data/local/tmp/kalibr_cam_chain_placeholder.yaml > /tmp/undistort_one.log 2>&1 &
sleep 12
echo "--- log ---"
tail -80 /tmp/undistort_one.log
echo "--- topics ---"
rostopic list | grep -E "undistort" || true
