#!/bin/bash
# watch_dog.sh

TIMEOUT=10  # 设定超时时间为10秒
NAMESPACE="" # 命名空间 例如 "/infantry_3" 注意要有"/"
NODE_NAMES=("armor_detector" "armor_solver" "serial_driver" "camera_driver")  # 列出所有需要监控的节点名称，注意是用空格分隔
USER="$(whoami)" #用户名
HOME_DIR=$(eval echo ~$USER)
WORKING_DIR="$HOME_DIR/fyt2024-main" # 代码目录
LAUNCH_FILE="rm_bringup bringup.launch.py" # launch 文件
OUTPUT_FILE="$WORKING_DIR/screen.output" # 终端输出记录文件

rmw="rmw_fastrtps_cpp" #RMW
export RMW_IMPLEMENTATION="$rmw" # RMW实现

export ROS_HOSTNAME=$(hostname)
export ROS_HOME=${ROS_HOME:=$HOME_DIR/.ros}
export ROS_LOG_DIR="/tmp"

source /opt/ros/humble/setup.bash
source $WORKING_DIR/install/setup.bash

rmw_config=""
if [[ "$rmw" == "rmw_fastrtps_cpp" ]]
then
  if [[ ! -z $rmw_config ]]
  then
    export FASTRTPS_DEFAULT_PROFILES_FILE=$rmw_config
  fi
elif [[ "$rmw" == "rmw_cyclonedds_cpp" ]]
then
  if [[ ! -z $rmw_config ]]
  then
    export CYCLONEDDS_URI=$rmw_config
  fi
fi

function bringup() {
    source /opt/ros/humble/setup.bash
    source $WORKING_DIR/install/setup.bash
    nohup ros2 launch $LAUNCH_FILE > "$OUTPUT_FILE" 2>&1 &
}

function restart() {
    pkill -f ros  # 杀掉所有ROS2进程
    ros2 daemon stop
    ros2 daemon start
    bringup
}

bringup
sleep $TIMEOUT
sleep $TIMEOUT

# 监控每个节点的心跳
while true; do
    for node in "${NODE_NAMES[@]}"; do
        topic="$NAMESPACE/$node/heartbeat"
        echo "- Check $node"
        if ros2 topic list 2>/dev/null | grep -q $topic 2>/dev/null; then
            data_value=$(timeout 10 ros2 topic echo $topic --once | grep -o "data: [0-9]*" | awk '{print $2}' 2>/dev/null)
            if [ ! -z "$data_value" ]; then
                echo "    $node is OK! Heartbeat Count: $data_value"
            else
                echo "    Heartbeat lost for $topic, restarting all nodes..."
                restart
                break 
            
            fi
        else
            echo "    Heartbeat topic $topic does not exist, restarting all nodes..."
            restart
            break
        fi
    done
    sleep $TIMEOUT
done

