#!/bin/bash
source /opt/ros/noetic/setup.bash
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"
ldd /data/local/tmp/libimage_undistort_nodelet.so | grep "not found" || echo "all dependencies resolved"
