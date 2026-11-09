#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
g++ -O2 -std=c++11 /tmp/omni_calib.cpp -o /usr/local/bin/omni_calib $(pkg-config --cflags --libs opencv4) -lopencv_ccalib
echo built
