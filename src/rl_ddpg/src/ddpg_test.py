#!/usr/bin/env python
# coding:utf-8
from ddpg_brain import DDPG
from ddpg_env import Env
import numpy as np
from tools import *

def run_task(env, net, max_episode,step_limit=100000,record_path="/data/test/data_example/record"):
    step = 0
    record_period = 1
    for episode in range(max_episode):
        print("reset")
        s = env.reset()
        print("test")        
        done = False
        if step > step_limit:      
            break
        if episode%record_period == 0:
            episode_record = Record('episode'+str(episode),record_path=record_path) # set Record

        # if step > net.memory_size: # Do not save net for test
        #     net.save()

        while done is False:
            a = net.choose_action(s)
            s_, reward, done = env.step(a)
            # net.store_transition(s, a, reward, s_)       # Do not train net for test         
            # if net.memory_counter > net.memory_size: 
            #     net.learn()

            # add record data 
            if episode%record_period == 0:
                episode_record.add(np.hstack((s, a, [reward], s_ ,env.BLDC3_state,[env.BLDC3_v],env.control_reward))) # add data

            s = s_
            step += 1
            if done:
                 # record 
                if episode%record_period == 0:
                    episode_record.save()  # save data 
                print("Episode: "+str(episode+1))
                print("Step: "+str(step+1))
                print("reward: "+str(reward))
                print("BLDC1： "+str(s[-14]))
                print("BLDC2： "+str(s[-4]))
                print("net.var: "+str(net.var))
                print("net.cost: "+str(net.c_cost))
                print("net.average_reward: "+str(net.average_reward))

if __name__ == '__main__':
    model = "202001111731"   # first version 202001081020
    model_path = "/data/train/" +model + "/model/"

    path = "/data/test/" + getTime()
    record_path = path + "/record/"
    create_test_dir(path)
    
    track_ddpg = DDPG(1, 23, np.array([12]),var_start=0.000000001,is_save=False, is_restore=True,restore_path=model_path)
    ####### for show net train results #############
    # track_ddpg.plot_a_cost()
    # track_ddpg.plot_c_cost()
    # track_ddpg.plot_average_reward()
    ######### select env #############################
    # track_env = Env(is_bldc2_control=False,is_compare=True)      # test for curve compare
    # track_env = Env(is_bldc2_control=True,is_compare=True)       # test for random motor compare

    # track_env = Env(is_bldc2_control=False,is_compare=False)     # test for curve single control
    track_env = Env(is_bldc2_control=True,is_compare=False)      # test for random motor single control

    run_task(track_env, track_ddpg,50,record_path=record_path)