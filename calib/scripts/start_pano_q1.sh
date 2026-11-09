#!/bin/bash
source /opt/ros/noetic/setup.bash
export OPENCV_OPENCL_RUNTIME=disabled
CONFIG=${1:-/data/local/tmp/kalibr_cam_chain_a4.yaml}
WIDTH=${2:-960}
HEIGHT=${3:-480}
METHOD=${4:-3}
LOG=${5:-/tmp/pano_q1.log}
FPS=${6:-15}
MAX_SKEW=${7:-0.20}
BLEND=${8:-0.10}
VFOV=${9:-90}
exec roslaunch /data/local/tmp/panorama_q1.launch config:=$CONFIG width:=$WIDTH height:=$HEIGHT method:=$METHOD fps:=$FPS max_skew:=$MAX_SKEW blend:=$BLEND vfov:=$VFOV > $LOG 2>&1
