#!/bin/bash
source /opt/ros/noetic/setup.bash
echo "--- nodes ---"
rosnode list
echo "--- params cam ---"
rosparam list | grep -E "cam[0-3]|concat" | head -80
