#!/usr/bin/env python
# coding=utf-8
import rospy
import numpy as np
import math
from std_msgs.msg import Float32


class Env(object):
    def __init__(self,is_compare=True,reset_speed_limit=20.0,reset_limit=5.0,s_bound=400.0,current_bound=20.0):
        # observation setting
        self.reset_limit = reset_limit
        self.reset_speed_limit = reset_speed_limit
        self.s_bound = s_bound
        self.s_bound2 = self.s_bound ** 2
        self.current_bound = current_bound
        self.BLDC3_state = np.zeros(10, dtype=np.float)
        self.BLDC2_state = np.zeros(10, dtype=np.float)
        self.BLDC3_current = 0.0
        self.BLDC2_current = 0.0
        self.BLDC3_v = 0.0
        self.BLDC2_v = 0.0
        self.ds = 0.0
        self.dv = 0.0
        # for compare 
        self.is_compare = is_compare   
        self.BLDC1_state = np.zeros(10, dtype=np.float)
        self.BLDC1_v = 0.0
        # ros setting
        rospy.init_node('track_controller')
        self.BLDC3_current_pub = rospy.Publisher('BLDC3_current', Float32, queue_size=100)
        self.control_reward_pub = rospy.Publisher('control_reward',Float32,queue_size=100)
        if not self.is_compare:
            self.BLDC2_current_pub = rospy.Publisher('BLDC2_current', Float32, queue_size=100)
        
        rospy.Subscriber('BLDC3_v', Float32, self.BLDC3_vCallback)
        rospy.Subscriber("BLDC3_s", Float32, self.BLDC3_sCallback)
        rospy.Subscriber('BLDC2_v', Float32, self.BLDC2_vCallback)
        rospy.Subscriber("BLDC2_s", Float32, self.BLDC2_sCallback)
        if self.is_compare:
            rospy.Subscriber('BLDC1_v', Float32, self.BLDC1_vCallback)
            rospy.Subscriber("BLDC1_s", Float32, self.BLDC1_sCallback)

        self.rate = rospy.Rate(50)
    
    def BLDC1_sCallback(self, data):
        BLDC1_state = np.append(self.BLDC1_state, data.data)
        self.BLDC1_state = BLDC1_state[-10:]

    def BLDC1_vCallback(self, data):
        self.BLDC1_v = data.data

    def BLDC3_sCallback(self, data):
        BLDC3_state = np.append(self.BLDC3_state, data.data)
        self.BLDC3_state = BLDC3_state[-10:]

    def BLDC2_sCallback(self, data):
        BLDC2_state = np.append(self.BLDC2_state, data.data)
        self.BLDC2_state = BLDC2_state[-10:]

    def BLDC3_vCallback(self, data):
        self.BLDC3_v = data.data

    def BLDC2_vCallback(self, data):
        self.BLDC2_v = data.data

    def pub_data(self):
        self.BLDC3_current_pub.publish(self.BLDC3_current)
        if not self.is_compare:
            self.BLDC2_current_pub.publish(self.BLDC2_current)

    def caculate_reward(self,d_current):
        reward_s = -math.fabs(self.BLDC3_state[-1] - self.BLDC2_state[-1])/10
        reward_v = -math.fabs(self.BLDC3_v-self.BLDC2_v)/50
        reward_ss = - np.clip(math.fabs(self.BLDC3_state[-1] - self.BLDC2_state[-1]) ** 2,0,10000)/500
        reward_c = - math.fabs(d_current)/20
        reward =  reward_s + reward_v + reward_ss + reward_c
        return reward

    def step(self, BLDC3_current):
        # origin setting 
        done = False
        observation = None

        # calculate current from action 
        BLDC3_d_current = BLDC3_current[0] - self.BLDC3_current
        self.BLDC3_current = BLDC3_current

        if not self.is_compare:
            self.BLDC2_current = self.BLDC2_current + np.random.normal(0, 2)
            if self.BLDC2_current > 20:
                self.BLDC2_current = 20
            elif self.BLDC2_current < -20:
                self.BLDC2_current = -20

        # pub current
        self.pub_data()
        if not rospy.is_shutdown():
            # step 0.02
            self.rate.sleep()

            # episode done judge
            if not self.is_compare:
                if self.BLDC3_state[-1] > self.s_bound or self.BLDC3_state[-1] < -self.s_bound:
                    done = True
                elif self.BLDC2_state[-1] > self.s_bound or self.BLDC2_state[-1] < -self.s_bound:
                    done = True
            else:
                if self.BLDC1_state[-1] > self.s_bound or self.BLDC1_state[-1] < -self.s_bound:
                    done = True
                elif self.BLDC2_state[-1] > self.s_bound or self.BLDC2_state[-1] < -self.s_bound:
                    done = True
            # caculate reward
            reward = self.caculate_reward(BLDC3_d_current)

            # get next observation 
            observation = np.concatenate([self.BLDC3_state, self.BLDC2_state])
            observation = np.append(observation,self.BLDC3_v)
            observation = np.append(observation,self.BLDC2_v)
            observation = np.append(observation,self.BLDC3_current)
            
            # pub reward
            self.control_reward_pub.publish(reward)
        self.get_dsdv()
        return observation, reward, done

    def caculate_reset_current(self):
        KP = 0.5
        kd = 0.2
        BLDC3_current = - KP * self.BLDC3_state[-1] - kd * self.BLDC3_v
        self.BLDC3_current = np.clip(BLDC3_current,-self.current_bound,self.current_bound)
        if not self.is_compare:
            BLDC2_current = - KP * self.BLDC2_state[-1] - kd * self.BLDC2_v
            self.BLDC2_current = np.clip(BLDC2_current,-self.current_bound,self.current_bound)

    def reset(self):
        done = False
        if self.is_compare:
            while (not rospy.is_shutdown()) and (done is not True):
                self.rate.sleep()
                if -self.reset_limit < self.BLDC1_state[-1] < self.reset_limit and -self.reset_limit < self.BLDC2_state[-1] <self.reset_limit:
                    if -self.reset_speed_limit< self.BLDC1_v < self.reset_speed_limit and -self.reset_speed_limit<self.BLDC2_v < self.reset_speed_limit:
                        done = True
                self.caculate_reset_current()
                self.pub_data()
            observation = np.concatenate([self.BLDC3_state, self.BLDC2_state])
            observation = np.append(observation,self.BLDC3_v)
            observation = np.append(observation,self.BLDC2_v)
            observation = np.append(observation,self.BLDC3_current)
        else: 
            while (not rospy.is_shutdown()) and (done is not True):
                self.rate.sleep()
                if -self.reset_limit < self.BLDC3_state[-1] < self.reset_limit and -self.reset_limit < self.BLDC2_state[-1] <self.reset_limit:
                    if -self.reset_speed_limit< self.BLDC3_v < self.reset_speed_limit and -self.reset_speed_limit<self.BLDC2_v < self.reset_speed_limit:
                        done = True
                self.caculate_reset_current()
                self.pub_data()
                observation = np.concatenate([self.BLDC3_state, self.BLDC2_state])
                observation = np.append(observation,self.BLDC3_v)
                observation = np.append(observation,self.BLDC2_v)
                observation = np.append(observation,self.BLDC3_current)
        return observation

    def get_dsdv(self):
        self.ds = self.BLDC3_state[-1] - self.BLDC2_state[-1]
        self.dv = self.BLDC3_v - self.BLDC2_v








