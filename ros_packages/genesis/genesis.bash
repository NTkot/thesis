#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

source /opt/ros/humble/setup.bash
source /home/rpi4/road_quality_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

pigpiod
sleep 0.5

sleep 5
if [ -z "$(nmcli con show --active)" ]
then
    echo "No wifi connection detected, starting own hotspot"
    nmcli dev wifi hotspot ifname wlan0 ssid RoadQualityWiFi password "roadquality"
fi

ros2 launch road_quality_core nucleus_launch.py

