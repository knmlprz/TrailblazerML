#!/bin/bash

cd ../src/ || exit 1
rosdep install --from-paths . --ignore-src -y -r
cd ../ || exit 1
colcon build
pwd
source ./install/setup.bash
source ./install/local_setup.bash
