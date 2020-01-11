#!/usr/bin/env python
# coding=utf-8

import numpy as np
import math

class sin_cruve:
    def __init__(self,bound=150,period=5):
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
    def __init__(self,bound=[-150,150],ramp=200,a=500,period=10):
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
    def __init__(self,bound=[-150,150],period=10):
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

