#!/usr/bin/env python
# coding:utf-8
from controller_env import Env
from controller import PID


def run_task(env,controller,max_episode,step_limit=100000):
    step = 0
    for episode in range(max_episode):
        print("reset")
        s = env.reset()
        controller.reset()
        print("control")
        done = False
        if step > step_limit:
            break
        while done is False:
            a = controller.calculate(s,0.02)
            s_, reward, done = env.step(a)
            s = s_
            step += 1

if __name__ == '__main__':
    track_env = Env(is_compare=True)
    PID_controller = PID(kp=1,ki=0,kd=0.2,dss_bound = 10)
    run_task(track_env,PID_controller,1000)