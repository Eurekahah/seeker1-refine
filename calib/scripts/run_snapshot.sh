#!/bin/bash
source /opt/ros/noetic/setup.bash
export OPENCV_OPENCL_RUNTIME=disabled
timeout 25 python3 /data/local/tmp/snapshot_pano.py /data/local/tmp/pano.jpg
echo "rc=$?"
ls -l /data/local/tmp/pano.jpg
