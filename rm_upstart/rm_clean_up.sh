#!/bin/bash

export ROS_HOSTNAME=$(hostname)
export ROS_HOME=${ROS_HOME:=$HOME_DIR/.ros}
export ROS_LOG_DIR="/tmp"
source /opt/ros/humble/setup.bash
pkill rm_watch_dog.sh # 杀掉看门狗
pkill -f ros  # 杀掉所有ROS2进程
ros2 daemon stop
ros2 daemon start
sleep 1
