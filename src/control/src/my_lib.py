# lib name: my lib.py
# author: Li Mingcong
# update: 2019.6.12
# instructions: provide functions for BLDC control and data processing.

import rospy
from std_msgs.msg import Float32
import time
import math
import numpy

# update speed series
def speed_series_update(original_series, speed_temp):
    '''A function to update the speed series.
    It returns the new series'''
    new_series = original_series
    new_series[:-1] = original_series[1:]
    new_series[-1] = speed_temp
    return new_series

# a filter based on the speed series
def speed_filter(speed_series):
    '''A function to update the speed based on speed series.
    It returns the new speed'''
    speed = numpy.average(speed_series)
    return speed

# Limitation function, enable user to setup a saturation function.
def Limitation(content, content_maximum):
    '''Putting a limitation on the output value, no more than the preset value'''
    content_limited = 0.0
    if (content >= -content_maximum and content <= content_maximum):
        content_limited = content
    elif (content < -content_maximum):
        content_limited = -content_maximum
    else:
        content_limited = content_maximum
    return content_limited

# BLDC Current Protection.
def BLDC_current_protect(current):
    '''BLDC current protection, no more than the preset value'''
    current_maximum = 15.0    # BLDC Maximum allowed current
    return Limitation(current, current_maximum)

# Angle Following Protection.
def Angle_Follow_protect(angle):
    '''Angle Following protection, no more than the preset value'''
    angle_maximum = 400    # Steering Wheel Maximum allowed angle
    return Limitation(angle, angle_maximum)

# load motor voltage calculation based on desired force (kN)
def loadMotor_voltage(desired_force):
    p_desire = 30.0    # preset in motor driver
    desired_voltage = desired_force*135.0/40.0/14.3*p_desire/10.0
    return desired_voltage

# clutch control protect
def clutch_state_protect(original_signal):
    if original_signal == 1:
        output_signal = 1
    else:
        output_signal = 0
    return output_signal


if __name__ == '__main__':
    print('Attention! This file is only for importing!')