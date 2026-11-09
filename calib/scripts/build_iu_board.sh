#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
mkdir -p /data/local/tmp/catkin_ws/src
rm -rf /data/local/tmp/catkin_ws/src/image_undistort
ln -s /data/local/tmp/image_undistort /data/local/tmp/catkin_ws/src/image_undistort
cd /data/local/tmp/catkin_ws
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release
cp -f devel/lib/libimage_undistort_nodelet.so /data/local/tmp/libimage_undistort_nodelet_cpu.so
ls -lh /data/local/tmp/libimage_undistort_nodelet_cpu.so
echo BUILD_OK
