#!/bin/bash
source /opt/ros/noetic/setup.bash
export OPENCV_OPENCL_RUNTIME=disabled
pkill -f panorama.launch 2>/dev/null || true
pkill -f test_manager 2>/dev/null || true
pkill -f 'nodelet standalone' 2>/dev/null || true
sleep 1
nohup roslaunch /data/local/tmp/panorama.launch config:=/data/local/tmp/kalibr_cam_chain_placeholder.yaml width:=320 height:=160 > /tmp/panorama.log 2>&1 &
echo $! > /tmp/panorama.pid
sleep 25
echo "--- log ---"
tail -60 /tmp/panorama.log
echo "--- nodes ---"
rosnode list
echo "--- topics ---"
rostopic list | grep panorama || true
