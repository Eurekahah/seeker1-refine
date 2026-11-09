#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
g++ -O2 -std=c++11 /tmp/fisheye_stereo.cpp -o /usr/local/bin/fisheye_stereo $(pkg-config --cflags --libs opencv4) -lopencv_ccalib
echo built
