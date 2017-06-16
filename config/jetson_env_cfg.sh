#!/usr/bin/env sh
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/ubuntu/inv/playground/pcl/workspace/src
export ROS_MASTER_URI=http://10.42.0.1:11311

 ./opt/ros/indigo/setup.sh
. /home/ubuntu/inv/playground/pcl/devel/setup.sh
exec "$@"
