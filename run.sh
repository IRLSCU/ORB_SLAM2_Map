#! /bin/bash
echo "sure! cur path is | ORB_SLAM2-Map/"
cur_path=`pwd`
ROS_PACKAGE_PATH_=$cur_path/Examples/ROS
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${ROS_PACKAGE_PATH_}
rosrun ORB_SLAM2 Stereo_tracking ./Vocabulary/ORBvoc.txt ./Examples/Stereo/mynteye_s_stereo_gps.yaml true
