# lmc_catkin_ws

This is for lmc testbench.


When using:

sudo chmod 777 /dev/ttyS1

sudo chmod 777 /dev/ttyS2

sudo chmod 777 /dev/ttyS3


//1

roscore


//2

source ./devel/setup.bash

rosrun usbcan usbcan_d


//3

source ./devel/setup.bash

rosrun drivers serial_


//4

source ./devel/setup.bash

rosrun testbench_interface testbench_interface

enjoy!!!
