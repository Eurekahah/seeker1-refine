#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
exec python3 /data/local/tmp/capture_4cam.py "$@"
