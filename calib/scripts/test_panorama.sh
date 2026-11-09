#!/bin/bash
source /opt/ros/noetic/setup.bash
pkill -f panorama.launch 2>/dev/null || true
sleep 1
nohup roslaunch /data/local/tmp/panorama.launch config:=/data/local/tmp/kalibr_cam_chain_placeholder.yaml > /tmp/panorama.log 2>&1 &
echo $! > /tmp/panorama.pid
sleep 15
echo "--- log ---"
tail -100 /tmp/panorama.log
echo "--- topics ---"
rostopic list | grep -E "panorama|concat" || true
