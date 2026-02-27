#!/bin/bash

mkdir voiture2A-ros -p
# shellcheck disable=SC2164
cd /voiture2A-ros
cp -r -p /home/src .

# clean install directory
#rm -r install

colcon build

echo "All done building"
