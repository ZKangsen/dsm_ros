#!/bin/bash

echo "Building ROS nodes"

cd ROS_Examples/ros_mono
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j