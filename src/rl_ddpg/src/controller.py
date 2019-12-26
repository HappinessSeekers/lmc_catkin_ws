#!/usr/bin/env python
# coding:utf-8
import numpy as np
from std_msgs.msg import Float32

class PID(object):
    def __init__(self,kp=0,ki=0,kd=0,dss_bound = 10):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.ds = 0
        self.dv = 0 
        self.dss = 0.0
        self.dss_bound = 10 

    def calculate(self,s,t):
        self.ds = s[19] - s[9]
        self.dv = s[21] - s[20] 
        self.dss = np.clip(self.dss + self.ds*t, -self.dss_bound,self.dss_bound)
        output = np.clip(self.ds * self.kp + self.dv * self.kd + self.dss * self.ki,-20,20)
        return np.array([output],dtype=Float32)

    def reset(self):
        self.dss = 0