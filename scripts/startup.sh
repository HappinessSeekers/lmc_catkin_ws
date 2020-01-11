#! /bin/bash

source /opt/ros/kinetic/setup.bash
sudo chmod 777 /dev/ttyS1
sudo chmod 777 /dev/ttyS2
sudo chmod 777 /dev/ttyS3
source ./devel/setup.bash
roslaunch startup testbench_interface.launch
