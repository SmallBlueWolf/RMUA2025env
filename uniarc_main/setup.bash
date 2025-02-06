#!/bin/bash
cd /uniarc_main
source devel/setup.bash
roslaunch controller_test controller_test.launch & \
wait