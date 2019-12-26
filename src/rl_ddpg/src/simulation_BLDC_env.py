#!/usr/bin/env python
'''simulation_env ROS Node'''
# license removed for brevity
import rospy
import numpy as np
import math
from std_msgs.msg import Float32


class BLDC:
    def __init__(self):
        self.s = 0.0
        self.v = 0.0
        self.current = 0.0
        self.ratio = 0.01
        self.friction_a0 = 1
        self.friction_a1 = 0.02
        self.currentMax = 20.0
        self.force = 0.0
        self.forceRatio = 2000
        self.current_force =0.0
    def set_current(self, current):
        if -self.currentMax < current < self.currentMax:
            self.current = current
        elif current >= self.currentMax:
            self.current = self.currentMax
        elif current <= -self.currentMax:
            self.current = -self.currentMax

    def step(self,time):
        self.s = self.s + self.v * time * 0.5
        self.current_force = self.force / self.forceRatio
        if self.v > 0:
            current_real = self.current-self.v*self.friction_a1 - self.friction_a0 - self.current_force
        elif self.v < 0:
            current_real = self.current-self.v*self.friction_a1 + self.friction_a0 - self.current_force
        elif self.v == 0:
            if -self.friction_a0 - self.current_force < self.current < self.friction_a0 - self.current_force:
                current_real = 0
            elif self.current >= (self.friction_a0 - self.current_force):
                current_real = self.current - self.friction_a0 - self.current_force
            else:
                current_real = self.current + self.friction_a0 - self.current_force
        self.v = self.v + current_real/self.ratio * time
        self.s = self.s + self.v * time * 0.5

    def s_measure(self):
        return self.s + np.random.uniform(0,0.5)
   
    def v_measure(self):
        return self.v + np.random.uniform(0,5)

class sin_cruve:
    def __init__(self,bound=200,period=5):
        self.bound = bound
        self.period = period
        self.t = 0.0
        self.s = 0.0
        self.v = 0.0 
        self.current = 0.0
    def cal_s(self):
        self.s = self.bound * math.sin(2 * math.pi *self.t / self.period)
    def cal_v(self):
        self.v = (self.bound * math.sin(2 * math.pi *self.t / self.period) - self.bound * math.sin(2 * math.pi *(self.t - 0.002) / self.period))/0.002
    def step(self,t):
        self.t = self.t + t
        self.cal_s()
        self.cal_v()

    def s_measure(self):
        return self.s

    def v_measure(self):
        return self.v

    def set_current(self, current):
        pass 

class step_cruve:
    def __init__(self,bound=[-200,200],ramp=400,a=1000,period=10):
        self.bound_low = bound[0]
        self.bound_high = bound[1]
        self.s = 0.0
        self.v = 0.0
        self.a = a
        self.ramp = ramp
        self.period = period
        self.t_period = 0.0
        self.t = 0.0
        self.current = 0.0
    def step(self, t):
        self.t = self.t + t
        self.t_period =  math.fmod(self.t,self.period)
        self.cal_v(t)
        self.cal_s(t)
    def cal_v(self,t):
        if (self.s < self.bound_high - self.ramp**2/2/self.a) and  (self.t_period < self.period / 2) :
            self.v = min(self.v + t * self.a,self.ramp)
        elif (self.s > self.bound_high - self.ramp**2/2/self.a) and  (self.t_period < self.period / 2) and self.v > 0:
            self.v =  self.v - t * self.a  
        elif  (self.s > self.bound_low + self.ramp**2/2/self.a) and  (self.t_period > self.period / 2):
            self.v = max(self.v - t * self.a,-self.ramp)
        elif (self.s < self.bound_low + self.ramp**2/2/self.a) and  (self.t_period > self.period / 2) and self.v < 0:
            self.v =  self.v + t * self.a  
        else:
            self.v = 0

    def cal_s(self,t):
        self.s = self.s + t*self.v

    def s_measure(self):
        return self.s

    def v_measure(self):
        return self.v

    def set_current(self, current):
        pass 
class step_cruve_s:
    def __init__(self,bound=[-200,200],period=10):
        self.bound_low = bound[0]
        self.bound_high = bound[1]
        self.s = 0.0
        self.v = 0.0
        self.t_period = 0.0
        self.period = period
        self.t = 0.0
        self.current = 0.0
    def step(self, t):
        self.t = self.t + t
        self.t_period =  math.fmod(self.t,self.period)
        self.cal_s()
    def cal_s(self):
        if self.t_period >  self.period*3/4:
            self.s = self.bound_low
        elif self.period*1/4 < self.t_period <  self.period*2/4:
            self.s = self.bound_high
        else:
            self.s = 0

    def s_measure(self):
        return self.s 

    def v_measure(self):
        return self.v

    def set_current(self, current):
        pass 
# BLDC1: active BLDC  
BLDC1 = BLDC()

# BLDC2: reference BLDC
BLDC2 = BLDC()

# BLDC3:compare BLDC
BLDC3 = BLDC()

# BLDC2 = step_cruve(bound=[-300,300],ramp=700,a=1000,period=5)
# BLDC2 = sin_cruve(period=3)
# BLDC2 = step_cruve_s(period=10)


def env_BLDC1_currentCallback(data):
    global BLDC1
    BLDC1.set_current(data.data)


def env_BLDC2_currentCallback(data):
    global BLDC2
    BLDC2.set_current(data.data)


def env_BLDC3_currentCallback(data):
    global BLDC3
    BLDC3.set_current(data.data)


def rackforce_recieverCallback(data):
    global BLDC1
    print("force: ",data.data)
    BLDC1.force = data.data


def main():
    rospy.init_node('simulation_env', anonymous=True)
    rate = rospy.Rate(500)
    count = 0
    count_max = 5
    global BLDC1
    global BLDC2
    global BLDC3
    # communication with brain
    env_BLDC1_s_pub = rospy.Publisher('env_BLDC1_s', Float32, queue_size=100)
    env_BLDC1_v_pub = rospy.Publisher('env_BLDC1_v', Float32, queue_size=100)

    env_BLDC2_s_pub = rospy.Publisher('env_BLDC2_s', Float32, queue_size=100)
    env_BLDC2_v_pub = rospy.Publisher('env_BLDC2_v', Float32, queue_size=100)

    env_BLDC3_s_pub = rospy.Publisher('env_BLDC3_s', Float32, queue_size=100)
    env_BLDC3_v_pub = rospy.Publisher('env_BLDC3_v', Float32, queue_size=100)

    env_BLDC12_ds_pub = rospy.Publisher('env_BLDC12_ds', Float32, queue_size=100)
    env_BLDC12_dv_pub = rospy.Publisher('env_BLDC12_dv', Float32, queue_size=100)

    env_BLDC32_ds_pub = rospy.Publisher('env_BLDC32_ds', Float32, queue_size=100)
    env_BLDC32_dv_pub = rospy.Publisher('env_BLDC32_dv', Float32, queue_size=100)

    rospy.Subscriber("env_BLDC1_current", Float32, env_BLDC1_currentCallback)
    rospy.Subscriber("env_BLDC2_current", Float32, env_BLDC2_currentCallback)
    rospy.Subscriber("env_BLDC3_current", Float32, env_BLDC3_currentCallback)
    # communication with matlab 
    roadWheelAngle_pub = rospy.Publisher("roadWheelAngle",Float32, queue_size=100)
    rospy.Subscriber("rackforce_feedback", Float32, rackforce_recieverCallback)

    while not rospy.is_shutdown():
        count = count + 1
        if count >= count_max:
            count = 0
            # # for sample data 
            # env_BLDC1_s_pub.publish(BLDC1.s_measure())
            # env_BLDC1_v_pub.publish(BLDC1.v_measure())

            # env_BLDC2_s_pub.publish(BLDC2.s_measure())
            # env_BLDC2_v_pub.publish(BLDC2.v_measure())

            # env_BLDC3_s_pub.publish(BLDC3.s_measure())
            # env_BLDC3_v_pub.publish(BLDC3.v_measure())
            # for real data 
            env_BLDC1_s_pub.publish(BLDC1.s)
            env_BLDC2_s_pub.publish(BLDC2.s)
            env_BLDC1_v_pub.publish(BLDC1.v)
            env_BLDC2_v_pub.publish(BLDC2.v)
            env_BLDC3_s_pub.publish(BLDC3.s)
            env_BLDC3_v_pub.publish(BLDC3.v)
            roadWheelAngle_pub.publish(BLDC1.s)

            BLDC12_ds = BLDC1.s - BLDC2.s
            BLDC12_dv = BLDC1.v - BLDC2.v
            BLDC32_ds = BLDC3.s - BLDC2.s
            BLDC32_dv = BLDC3.v - BLDC2.v

            env_BLDC12_ds_pub.publish(BLDC12_ds)
            env_BLDC12_dv_pub.publish(BLDC12_dv)
            env_BLDC32_ds_pub.publish(BLDC32_ds)
            env_BLDC32_dv_pub.publish(BLDC32_dv)

            ######## print list ################
            # print("env_BLDC1_s: "+str(BLDC1.s))
            # print("BLDC1_V: "+str(BLDC1.v))
            # print("BLDC1_C: "+str(BLDC1.current))
            # print("BLDC2_s: "+str(BLDC2.s))
            # print("BLDC2_V: "+str(BLDC2.v))
            # print("BLDC2_C: "+str(BLDC2.current))
            # print("BLDC3_s: "+str(BLDC3.s))
            # print("BLDC3_V: "+str(BLDC3.v))
            # print("BLDC3_C: "+str(BLDC3.current))

        BLDC1.step(0.002)
        BLDC2.step(0.002)
        BLDC3.step(0.002)
        rate.sleep()


if __name__ == '__main__':
    main()
