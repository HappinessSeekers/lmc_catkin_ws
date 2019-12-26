#! /bin/bash

source ./devel/setup.bash
conda activate py36
roslaunch startup rl_ddpg_sim_train.launch

