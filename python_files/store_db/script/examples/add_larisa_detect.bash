#!/bin/bash

DIRPATH=$(dirname $(realpath "$0"))

python3 -m store_db.script.script store \
    --data /home/ntkot/theses/ros/bag_db/data_07_10_2023__17_42_10/rosbag2_07_10_2023__17_42_10/rosbag2_07_10_2023__17_42_10.pkl \
    --preprocess
