#!/bin/bash

DIRPATH=$(dirname $(realpath "$0"))

python3 -m store_db.script.script store \
    --data /home/ntkot/theses/ros/bag_db/data_01_10_2023__17_48_46/split1/split1.pkl \
    --options $DIRPATH/salouga.yaml \
    --preprocess
