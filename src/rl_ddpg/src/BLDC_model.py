#!/usr/bin/env python
# coding=utf-8
import numpy as np
import math

class BLDC:
    def __init__(self):
        self.s = 0.0
        self.v = 0.0
        self.current = 0.0
        self.ratio = 0.02
        self.friction_a0 = 1
        self.friction_a1 = 0.015
        self.currentMax = 20.0
        self.force = 0.0
        self.forceRatio = -0.5
        self.current_force =0.0
        self.force_count = 0 # for print count

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
