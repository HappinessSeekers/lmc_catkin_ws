#!/usr/bin/env python
# coding:utf-8
from controller_env import Env
from controller import PID
import numpy as np
from tools import *

def run_task(env,controller,max_episode,step_limit=100000,record_path="/data/control/data_example/record"):
    step = 0
    record_period = 1
    for episode in range(max_episode):
        print("reset")
        s = env.reset()
        controller.reset()
        print("control")
        done = False
        if step > step_limit:
            break
        if episode%record_period == 0:
            episode_record = Record('episode'+str(episode),record_path=record_path) # set Record

        while done is False:
            a = controller.calculate(s,0.02)
            s_, reward, done = env.step(a)
            # add record data 
            if episode%record_period == 0:
                episode_record.add(np.hstack((s, a, [reward], s_))) # add data
            s = s_
            step += 1
            if done:
                 # record 
                if episode%record_period == 0:
                    episode_record.save()  # save data 

if __name__ == '__main__':

    path = "/data/control/" + getTime()
    record_path = path + "/record/"
    create_dir(path)

    PID_controller = PID(kp=1,ki=0,kd=0.2,dss_bound = 10)
    ##### select env #################
    # track_env = Env(is_bldc2_control=False,is_compare=True)  # for random cruve compare 
    # track_env = Env(is_bldc2_control=True,is_compare=True)   # for random motor compare

    # track_env = Env(is_bldc2_control=False,is_compare=False) # for random curve single control 
    track_env = Env(is_bldc2_control=True,is_compare=False)  # for random motor single control 

    run_task(track_env,PID_controller,500,record_path=record_path)
