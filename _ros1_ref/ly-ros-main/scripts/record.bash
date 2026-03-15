#!/bin/bash

ROOT_DIR="/home/hustlyrm"

RECORD_DIR="${ROOT_DIR}/workspace/record"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# 原数据包文件名
FILENAME="${RECORD_DIR}/record_${TIMESTAMP}.bag"
# 新增图像数据包文件名
FILENAME_IMAGE="${RECORD_DIR}/record_${TIMESTAMP}_image.bag"

python3 ${ROOT_DIR}/workspace/scripts/throttle_node.py &

sleep 1

# 启动两个独立录制进程
rosbag record -O $FILENAME /ly/gimbal/angles /ly/predictor/target /ly/tracker/results /ly/buff/target /ly/control/angles /ly/detector/armors /ly/bt/target &
rosbag record -O $FILENAME_IMAGE  /ly/ra/angle_image_throttled &


echo "已保存记录文件："
echo "- 主数据包：$FILENAME" 
echo "- 图像数据包：$FILENAME_IMAGE"