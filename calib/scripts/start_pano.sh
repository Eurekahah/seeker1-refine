#!/bin/bash
source /opt/ros/noetic/setup.bash
export OPENCV_OPENCL_RUNTIME=disabled
CONFIG=${1:-/data/local/tmp/kalibr_cam_chain_placeholder.yaml}
WIDTH=${2:-320}
HEIGHT=${3:-160}
exec roslaunch /data/local/tmp/panorama.launch config:=$CONFIG width:=$WIDTH height:=$HEIGHT
