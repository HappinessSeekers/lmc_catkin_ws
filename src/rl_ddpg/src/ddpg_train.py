#!/usr/bin/env python
# coding:utf-8
from ddpg_brain import DDPG
from ddpg_env import Env
import numpy as np


def run_task(env, net, max_episode,step_limit=100000):
    step = 0
    for episode in range(max_episode):
        print("reset")
        s = env.reset()
        print("train")        
        done = False
        if step > step_limit:
            break
        if step > net.memory_size:
            net.save()

        while done is False:
            a = net.choose_action(s)
            s_, reward, done = env.step(a)
            net.store_transition(s, a, reward, s_)            
            if net.memory_counter > net.memory_size:
                net.learn()
            s = s_
            print(a)
            print(s)
            print(reward)
            step += 1
            if done:
                print("Episode: "+str(episode+1))
                print("Step: "+str(step+1))
                print("reward: "+str(reward))
                print("BLDC1： "+str(s[-14]))
                print("BLDC2： "+str(s[-4]))
                print("net.var: "+str(net.var))
                print("net.cost: "+str(net.c_cost))
                print("net.average_reward: "+str(net.average_reward))

if __name__ == '__main__':
    #### train from origen ######
    track_ddpg = DDPG(1, 23, np.array([10]),is_save=True, is_restore=False)
    track_env = Env()
    run_task(track_env,track_ddpg,1500,step_limit=32*track_ddpg.memory_size)

    ##### continue train #######
    # track_ddpg = DDPG(1, 23, np.array([20]),var_start=0.05,is_save=True, is_restore=True)
    # track_env = Env()
    # run_task(track_env, track_ddpg,500,step_limit=16*track_ddpg.memory_size)

    ##### replay #######
    # track_ddpg = DDPG(1, 23, np.array([20]),var_start=0.000000001,is_save=False, is_restore=True)
    # track_ddpg.plot_a_cost()
    # track_ddpg.plot_c_cost()
    # track_ddpg.plot_average_reward()
    # track_env = Env()
    # run_task(track_env, track_ddpg,100,step_limit=track_ddpg.memory_size)

    ##### replay without graph #######
    # track_ddpg = DDPG(1, 23, np.array([20]),var_start=0.000000001,is_save=False, is_restore=True)
    # track_env = Env()
    # run_task(track_env, track_ddpg,100,step_limit=track_ddpg.memory_size)
