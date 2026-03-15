#!/bin/bash
source /opt/ros/noetic/setup.bash

# 持续向两个话题发送false值（每秒1次）
rostopic pub -r 1 /ly/aa/enable std_msgs/Bool "data: false" &
rostopic pub -r 1 /ly/ra/enable std_msgs/Bool "data: true" &

# 保持脚本运行，直到用户手动终止（如Ctrl+C）
wait
