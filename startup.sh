#!/bin/bash
clear
echo "Launching datalogger..."
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
source /etc/ubiquity/env.sh
roslaunch /home/ubuntu/catkin_ws/src/div_datalogger/launch/div_datalogger.launch
