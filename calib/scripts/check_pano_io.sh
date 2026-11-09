#!/bin/bash
source /opt/ros/noetic/setup.bash
echo "--- concat connections ---"
rosnode info /concat_node 2>&1 | head -100
echo "--- raw rates ---"
python3 /data/local/tmp/count_images.py
