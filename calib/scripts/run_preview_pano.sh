#!/bin/bash
source /opt/ros/noetic/setup.bash
export OPENCV_OPENCL_RUNTIME=disabled
exec python3 /data/local/tmp/preview_panorama.py "$@"
