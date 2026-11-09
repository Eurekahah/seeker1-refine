#!/bin/bash
source /opt/ros/noetic/setup.bash
export OPENCV_OPENCL_RUNTIME=disabled
CONFIG=${1:-/data/local/tmp/kalibr_cam_chain.yaml}
WIDTH=${2:-960}
HEIGHT=${3:-480}
METHOD=${4:-0}
LOG=${5:-/tmp/pano_slow.log}
exec roslaunch /data/local/tmp/panorama_slow.launch config:=$CONFIG width:=$WIDTH height:=$HEIGHT method:=$METHOD > $LOG 2>&1