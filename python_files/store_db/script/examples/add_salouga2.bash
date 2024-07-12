#!/bin/bash

DIRPATH=$(dirname $(realpath "$0"))

python3 -m store_db.script.script store \
    --data /home/ntkot/theses/ros/bag_db/data_19_11_2023__11_50_12/rosbag2_19_11_2023__11_50_12/rosbag2_19_11_2023__11_50_12.pkl \
    --options $DIRPATH/salouga2.yaml \
    --preprocess
