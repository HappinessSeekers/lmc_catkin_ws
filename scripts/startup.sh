#! /bin/bash

source /opt/ros/kinetic/setup.bash
chmod 777 /dev/ttyS1
chmod 777 /dev/ttyS2
chmod 777 /dev/ttyS3
source ./devel/setup.bash
# roslaunch startup test.launch
