#!/bin/bash
source /opt/ros/noetic/setup.bash
pkill -f test_manager 2>/dev/null || true
pkill -f test_concat 2>/dev/null || true
sleep 1
rosrun nodelet nodelet manager __name:=test_manager > /tmp/test_manager.log 2>&1 &
sleep 2
rosparam load /data/local/tmp/kalibr_cam_chain_placeholder.yaml /test_concat
rosparam set /test_concat/input_camera_info_from_ros_params true
rosparam set /test_concat/first_camera_namespace cam0
rosparam set /test_concat/second_camera_namespace cam1
rosparam set /test_concat/third_camera_namespace cam2
rosparam set /test_concat/fourth_camera_namespace cam3
rosparam set /test_concat/concat_method 3
rosparam set /test_concat/concat_width 640
rosparam set /test_concat/concat_height 320
rosparam set /test_concat/blend_weight 0.4
rosparam set /test_concat/pano_radius 300
rosparam set /test_concat/approximate_sync true
echo "--- load nodelet ---"
rosrun nodelet nodelet load image_undistort/ImageConcatNodelet test_manager __name:=test_concat
echo "load rc=$?"
echo "--- manager log ---"
tail -120 /tmp/test_manager.log
