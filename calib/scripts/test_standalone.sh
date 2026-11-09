#!/bin/bash
source /opt/ros/noetic/setup.bash
rosparam load /data/local/tmp/kalibr_cam_chain_placeholder.yaml /test
rosparam set /test/input_camera_info_from_ros_params true
rosparam set /test/input_camera_namespace cam0
rosparam set /test/output_camera_info_source match_input
rosparam set /test/undistort_image true
timeout 8 rosrun nodelet nodelet standalone image_undistort/ImageUndistortNodelet __name:=test
echo "rc=$?"
