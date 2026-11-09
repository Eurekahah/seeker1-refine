#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
export LD_LIBRARY_PATH="/catkin_ws/src/seeker1/libs/x86${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec rosrun web_video_server web_video_server _port:=8080
