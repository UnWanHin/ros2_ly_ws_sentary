#!/bin/bash
# filepath: /home/liu/workspace/scripts/start_shooting_table_calib.bash

source devel/setup.bash

echo "Starting Shooting Table Calibration System..."
echo "============================================="

# 创建日志目录
# mkdir -p /home/liu/workspace/logs/shooting_table_calib/log
# mkdir -p /home/liu/workspace/logs/gimbal_driver/log

bash param_manager.bash &
PARAM_MANAGER_PID=$!
# 启动射击表校准系统
roslaunch shooting_table_calib shooting_table_calib.launch

kill -9 $PARAM_MANAGER_PID 2>/dev/null
echo "Shooting Table Calibration System stopped."