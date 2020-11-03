#!/bin/sh
echo "1/5 Launch Realsense"
xterm  -e "source ~/ros2_overlay_ws/install/setup.bash; ros2 run realsense_node realsense_node __params:=`ros2 pkg prefix realsense_examples`/share/realsense_examples/config/d435.yaml" &
sleep 5

echo "2/5 Bring up calibration board detection"
xterm  -e "ros2 launch handeye_target_detection pose_estimation.launch.py" &
sleep 5

echo "3/5 Bring up UR5 robot"
xterm  -e " ros2 launch robot_interface ur_test.launch.py move:=false" &
sleep 5

echo "4/5 Launch Rviz"
xterm  -e " ros2 launch ur_description view_ur5_ros2.launch.py" &
sleep 5

echo "5/5 Bring up calibration dashboard"
xterm  -e "ros2 launch handeye_dashboard handeye_dashboard.launch.py" 
