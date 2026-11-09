#!/bin/bash
source /opt/ros/noetic/setup.bash
echo "--- topics ---"
rostopic list
echo "--- cam left info ---"
rostopic echo -n1 /fisheye/left/camera_info
echo "--- hz left ---"
timeout 4 rostopic hz /fisheye/left/image_raw
