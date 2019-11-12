#! /bin/bash
set -e
echo "in ORB_SLAM2-MAP"
cur_path=`pwd`
ROS_PACKAGE_PATH_=$cur_path/Examples/ROS
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${ROS_PACKAGE_PATH_}
./build.sh
./build_ros.sh
echo "!!!ok!!!"
