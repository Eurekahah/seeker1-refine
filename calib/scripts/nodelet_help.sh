#!/bin/bash
source /opt/ros/noetic/setup.bash
rosrun nodelet nodelet --help 2>&1 | head -60
