#!/bin/bash

DIRPATH=$(dirname $(realpath "$0"))

python3 -m store_db.script.script store \
    --data /home/ntkot/Desktop/theses/ros/bag_db/data_07_07_2023__20_18_13/split1/split1_inj.pkl \
    --options $DIRPATH/salouga3_a.yaml \
    --preprocess

python3 -m store_db.script.script store \
    --data /home/ntkot/Desktop/theses/ros/bag_db/data_07_07_2023__20_18_13/split2/split2_inj.pkl \
    --options $DIRPATH/salouga3_b.yaml \
    --preprocess
