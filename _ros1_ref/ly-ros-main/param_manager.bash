#!/bin/bash
while true; do
    rosparam load src/detector/config/auto_aim_config.yaml
    echo "Configuration parameters reloaded."
    sleep 2
done