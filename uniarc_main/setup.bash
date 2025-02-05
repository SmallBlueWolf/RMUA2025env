#!/bin/bash
cd /uniarc_main
source devel/setup.bash
rosrun imu_gps_odometry imu_gps_odometry && \
rosrun controller_test controller_test