#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
cd /
tar -xzf /data/local/tmp/image_undistort_aarch64.tar.gz
echo "--- installed files ---"
ls -l /opt/ros/noetic/lib/libimage_undistort_nodelet.so
ls /opt/ros/noetic/share/image_undistort
echo "--- rospack ---"
rospack find image_undistort
echo "--- nodelet plugin ---"
rospack plugins --attrib=plugin nodelet | grep image_undistort || true
