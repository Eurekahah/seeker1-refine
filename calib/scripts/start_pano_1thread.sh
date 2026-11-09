#!/bin/bash
source /opt/ros/noetic/setup.bash
export OPENCV_OPENCL_RUNTIME=disabled
export MALLOC_ARENA_MAX=1
export MALLOC_TRIM_THRESHOLD_=131072
export MALLOC_MMAP_THRESHOLD_=131072
CONFIG=${1:-/data/local/tmp/kalibr_cam_chain.yaml}
WIDTH=${2:-960}
HEIGHT=${3:-480}
METHOD=${4:-3}
LOG=${5:-/tmp/pano_run.log}
exec roslaunch /data/local/tmp/panorama_1thread.launch config:=$CONFIG width:=$WIDTH height:=$HEIGHT method:=$METHOD > $LOG 2>&1