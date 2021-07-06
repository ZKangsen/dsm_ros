#!/bin/bash

img_topic=$1
if [ "$img_topic" = "" ]; then
    echo "please input image topic..."
    exit
fi
rosrun ros_mono ros_mono $img_topic ./ROS_Examples/calib_file/self_calib.txt ./ROS_Examples/calib_file/setting.txt
