#!/bin/bash
source /opt/ros/noetic/setup.bash
python3 - <<'PY'
import ctypes
try:
    ctypes.CDLL('/opt/ros/noetic/lib/libimage_undistort_nodelet.so')
    print('dlopen OK')
except OSError as e:
    print('dlopen FAILED:', e)
PY
