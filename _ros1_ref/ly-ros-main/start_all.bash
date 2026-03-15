#!/bin/bash

source ./devel/setup.bash

# bash /home/hustlyrm/workspace/scripts/start_gimbal_driver.bash &
# bash /home/hustlyrm/workspace/scripts/start_behavior_tree.bash &
# bash /home/hustlyrm/workspace/scripts/start_armor_detector.bash &
# bash /home/hustlyrm/workspace/scripts/start_armor_predictor.bash &

echo 1 > /home/hustlyrm/workspace/devel/lib/behavior_tree/.waiting_for_begin
roslaunch detector auto_aim.launch &


