#!/usr/bin/env python
# coding=utf-8
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
import sys
import csv

class Env(object):
    def __init__(self,reset_limit=10.0,reset_speed_limit=20.0,s_bound=150.0,current_bound=7.0):
        # observation setting        
        self.reset_limit = reset_limit
        self.reset_speed_limit = reset_speed_limit
        self.s_bound = s_bound
        self.s_bound2 = self.s_bound ** 2
        self.current_bound = current_bound
        self.BLDC1_state = np.zeros(10, dtype=np.float)
        self.BLDC2_state = np.zeros(10, dtype=np.float)
        self.BLDC1_current = 0.0
        self.BLDC2_current = 0.0
        self.BLDC1_v = 0.0
        self.BLDC2_v = 0.0
        self.ds = 0.0
        self.dv = 0.0
        # ros setting
        rospy.init_node('track_ddpg')
        self.BLDC1_current_pub = rospy.Publisher('env_BLDC1_current', Float32, queue_size=100)
        self.BLDC2_current_pub = rospy.Publisher('env_BLDC2_current', Float32, queue_size=100)
        self.ddpg_reward_pub = rospy.Publisher('ddpg_reward',Float32,queue_size=100)

        rospy.Subscriber('env_BLDC1_v', Float32, self.BLDC1_vCallback)
        rospy.Subscriber('env_BLDC2_v', Float32, self.BLDC2_vCallback)
        rospy.Subscriber("env_BLDC1_s", Float32, self.BLDC1_sCallback)
        rospy.Subscriber("env_BLDC2_s", Float32, self.BLDC2_sCallback)
        self.rate = rospy.Rate(50)
        
    def BLDC1_sCallback(self, data):
        BLDC1_state = np.append(self.BLDC1_state, data.data)
        self.BLDC1_state = BLDC1_state[-10:]

    def BLDC2_sCallback(self, data):
        BLDC2_state = np.append(self.BLDC2_state, data.data)
        self.BLDC2_state = BLDC2_state[-10:]

    def BLDC1_vCallback(self, data):
        self.BLDC1_v = data.data

    def BLDC2_vCallback(self, data):
        self.BLDC2_v = data.data

    def pub_data(self):
        self.BLDC1_current_pub.publish(self.BLDC1_current)
        self.BLDC2_current_pub.publish(self.BLDC2_current)

    def caculate_reward(self,d_current):
        reward_s = -math.fabs(self.BLDC1_state[-1] - self.BLDC2_state[-1])/10
        reward_v = -math.fabs(self.BLDC1_v-self.BLDC2_v)/50
        reward_ss = - np.clip(math.fabs(self.BLDC1_state[-1] - self.BLDC2_state[-1]) ** 2,0,10000)/200
        reward_c = - math.fabs(d_current)/10
        reward =  reward_s + reward_v + reward_ss + reward_c
        return reward

    def step(self, BLDC1_current):
        # origin setting 
        done = False
        observation = None

        # calculate current from action 
        BLDC1_d_current = BLDC1_current[0] - self.BLDC1_current
        self.BLDC1_current = BLDC1_current
        self.BLDC2_current = self.BLDC2_current + np.random.normal(0, 1)
        if self.BLDC2_current > self.current_bound:
            self.BLDC2_current = self.current_bound
        elif self.BLDC2_current < -self.current_bound:
            self.BLDC2_current = -self.current_bound

        # pub current 
        self.pub_data()
        if not rospy.is_shutdown():
            # step 0.02
            self.rate.sleep()

            # episode done judge
            if self.BLDC1_state[-1] > self.s_bound or self.BLDC1_state[-1] < -self.s_bound:
                done = True
            elif self.BLDC2_state[-1] > self.s_bound or self.BLDC2_state[-1] < -self.s_bound:
                done = True

            # caculate reward
            reward = self.caculate_reward(BLDC1_d_current)

            # get next observation 
            observation = np.concatenate([self.BLDC1_state, self.BLDC2_state])
            observation = np.append(observation,self.BLDC1_v)
            observation = np.append(observation,self.BLDC2_v)
            observation = np.append(observation,self.BLDC1_current)

            # pub reward
            self.ddpg_reward_pub.publish(reward)
        self.get_dsdv()
        return observation, reward, done

    def caculate_reset_current(self):
        Kp = 0.2
        kd = 0
        BLDC1_current = - Kp * self.BLDC1_state[-1] - kd * self.BLDC1_v
        BLDC2_current = - Kp * self.BLDC2_state[-1] - kd * self.BLDC2_v
        self.BLDC1_current = np.clip(BLDC1_current,-self.current_bound,self.current_bound)
        self.BLDC2_current = np.clip(BLDC2_current,-self.current_bound,self.current_bound)

    def reset(self):
        done = False
        while (not rospy.is_shutdown()) and (done is not True):
            self.rate.sleep()
            if -self.reset_limit < self.BLDC1_state[-1] < self.reset_limit and -self.reset_limit < self.BLDC2_state[-1] <self.reset_limit:
                if -self.reset_speed_limit< self.BLDC1_v < self.reset_speed_limit and -self.reset_speed_limit<self.BLDC2_v < self.reset_speed_limit:
                    done = True
            self.caculate_reset_current()
            self.pub_data()
            
        # get next observation
        observation = np.concatenate([self.BLDC1_state, self.BLDC2_state])
        observation = np.append(observation,self.BLDC1_v)
        observation = np.append(observation,self.BLDC2_v)
        observation = np.append(observation,self.BLDC1_current)
        return observation

    def get_dsdv(self):
        self.ds = self.BLDC1_state[-1] - self.BLDC2_state[-1]
        self.dv = self.BLDC1_v - self.BLDC2_v



class Record(object):
    def __init__(self,name,record_path="/data/record/"):
        self.record_path = sys.path[0] + record_path + name + ".csv"
        self.name = name 
        self.dataList = []
    def add(self,data):
        self.dataList.append(data)
    def clear(self):
        self.dataList = []
    def save(self):
        with open(self.record_path,'w') as f:
            f_csv = csv.writer(f)
            f_csv.writerows(self.dataList)




