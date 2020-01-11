#!/usr/bin/env python
# coding:utf-8
from ddpg_brain import DDPG
from ddpg_env import Env
import numpy as np
from tools import *

def run_task(env, net, max_episode,step_limit=100000,record_path="/data/train/data_example/record"):
    step = 0
    record_period = 1

    for episode in range(max_episode):
        print("reset")
        s = env.reset()
        print("train")        
        done = False
        # do after reset
        if step > step_limit:
            break
        if step > net.memory_size:
            net.save()
        if episode%record_period == 0:
            episode_record = Record('episode'+str(episode),record_path=record_path) # set Record

        while done is False:
            a = net.choose_action(s)
            s_, reward, done = env.step(a)
            net.store_transition(s, a, reward, s_)            
            if net.memory_counter > net.memory_size:
                net.learn()

            # add record data 
            if episode%record_period == 0:
                episode_record.add(np.hstack((s, a, [reward], s_, [env.rackforce_feedback]))) # add data

            # change state 
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

    path = "/data/train/" + getTime()
    model_path = path + "/model/"
    record_path = path + "/record/"
    create_test_dir(path)
    
    # #### train from origen ######
    # track_ddpg = DDPG(1, 23, np.array([10]),is_save=True, is_restore=False,save_path=model_path)
    # track_env = Env()
    # run_task(track_env,track_ddpg,1500,step_limit=32*track_ddpg.memory_size,record_path=record_path)

    ##### continue train #######

    old_model_path = "/data/train/"+"202001111706"+"/model/"
    track_ddpg = DDPG(1, 23, np.array([12]),var_start=0.05,is_save=True,lr_a=0.002,lr_c=0.004,is_restore=True,save_path=model_path,restore_path=old_model_path)
    track_env = Env()
    run_task(track_env, track_ddpg,500,step_limit=16*track_ddpg.memory_size,record_path=record_path)

