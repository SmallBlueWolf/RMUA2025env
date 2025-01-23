#!/bin/bash
cd /uniarc_main
source devel/setup.bash
# rosrun vins vins_node /uniarc_main/configs/uniarc_vins.yaml &
rosrun imu_gps_odometry imu_gps_odometry &
rosrun uniarc_main uniarc_main &
wait