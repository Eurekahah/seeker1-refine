#!/bin/bash
source /opt/ros/noetic/setup.bash
echo "--- nodes ---"
rosnode list
echo "--- concat info ---"
rosnode info /concat_node 2>&1 | head -80
echo "--- log tail ---"
tail -20 /tmp/panorama.log
