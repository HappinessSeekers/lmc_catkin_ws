#!/usr/bin/env python
'''simulation_env ROS Node'''
# license removed for brevity
import rospy
import numpy as np
import math
from std_msgs.msg import Float32

class BLDC_real:
    def __init__(self):
        self.s = 0
        self.v = 0
        self.current = 0

    def calculate(self,new_s,dt = 0.01):
        new_v = (new_s-self.s) / dt
        self.s = new_s 
        self.v = self.v * 0.8 +  new_v * 0.2


class load_Motor:
    def __init__(self,p_desire=30.0):
        self.voltage = 0.0  # v
        self.force = 0.0   # kN
        self.p_desire = p_desire
    def calculate(self,force):
        self.force = force 
        self.voltage = self.force*135.0/40.0/14.3*self.p_desire/10.0
        return self.voltage

class ROS_env():
    def __init__(self):

        rospy.init_node('testbench_env', anonymous=True)
        self.rate = rospy.Rate(500)
        self.steerwheelMotor =  BLDC_real()
        self.roadwheelMotor =  BLDC_real()
        self.loadMotor = load_Motor()

        # control count 
        self.count = 0
        self.count_max = 50

        # communication with brain
        self.env_BLDC1_s_pub = rospy.Publisher('env_BLDC1_s', Float32, queue_size=100) # env_BLDC1: road_wheel
        self.env_BLDC1_v_pub = rospy.Publisher('env_BLDC1_v', Float32, queue_size=100)

        self.env_BLDC2_s_pub = rospy.Publisher('env_BLDC2_s', Float32, queue_size=100) # env_BLDC2: steer_wheel
        self.env_BLDC2_v_pub = rospy.Publisher('env_BLDC2_v', Float32, queue_size=100)

        self.env_BLDC12_ds_pub = rospy.Publisher('env_BLDC12_ds', Float32, queue_size=100)
        self.env_BLDC12_dv_pub = rospy.Publisher('env_BLDC12_dv', Float32, queue_size=100)

        rospy.Subscriber("env_BLDC1_current", Float32, self.env_BLDC1_currentCallback)
        rospy.Subscriber("env_BLDC2_current", Float32, self.env_BLDC2_currentCallback)

        # communication with test bench 
        rospy.Subscriber("roadWheelAngle", Float32, self.roadwheel_angle_reciever_callback)
        rospy.Subscriber("steerWheelAngle", Float32, self.steerwheel_angle_reciever_callback)
        rospy.Subscriber("BLDC0_current_feedback", Float32, self.BLDC0_current_reciever_callback)
        rospy.Subscriber("BLDC1_current_feedback", Float32, self.BLDC1_current_reciever_callback)

        self.loadMotor_pub = rospy.Publisher('loadMotor', Float32, queue_size=10)
        self.clutch_pub = rospy.Publisher('clutch', Float32, queue_size=10)
        self.BLDC0_current_pub = rospy.Publisher('BLDC0_current', Float32, queue_size=10)
        self.BLDC1_current_pub = rospy.Publisher('BLDC1_current', Float32, queue_size=10)

        # communication with matlab 
        rospy.Subscriber("rackforce_feedback", Float32, self.rackforce_reciever_callback)
        rospy.Subscriber("windows_matlab_cmd", Float32, self.windows_matlab_cmd_callback)
        self.windows_matlab_response_pub = rospy.Publisher('windows_matlab_response', Float32, queue_size=10)



    def mainloop(self):
        while not rospy.is_shutdown():
            self.count = self.count + 1
            if self.count >= self.count_max:
                self.count = 0
                self.clutch_pub.publish(1)                
            self.rate.sleep()

    # transimit control data 
    def env_BLDC1_currentCallback(self,data):
        self.roadwheelMotor.current = data.data
        print("roadwheelMotor.current: ",self.roadwheelMotor.current)
        self.BLDC0_current_pub.publish(self.roadwheelMotor.current)


    def env_BLDC2_currentCallback(self,data):
        self.steerwheelMotor.current = -data.data
        print("steerwheelMotor.current: ",self.steerwheelMotor.current)
        self.BLDC1_current_pub.publish(self.steerwheelMotor.current)

    # transimit state data
    def roadwheel_angle_reciever_callback(self,data):
        print("roadwheel_angle: ",data.data)
        self.roadwheelMotor.calculate(data.data,dt=0.01)
        self.env_BLDC1_s_pub.publish(self.roadwheelMotor.s)
        self.env_BLDC1_v_pub.publish(self.roadwheelMotor.v)
        # print(self.roadwheelMotor.s)

    def steerwheel_angle_reciever_callback(self,data):
        print("steerwheel_angle: ",data.data)
        self.steerwheelMotor.calculate(data.data,dt=0.01)
        self.env_BLDC2_s_pub.publish(self.steerwheelMotor.s)
        self.env_BLDC2_v_pub.publish(self.steerwheelMotor.v)
        # print(self.steerwheelMotor.s)

    def rackforce_reciever_callback(self,data):
        # print("rackforce_reciever: ",data.data)
        # self.loadMotor_pub.publish(self.loadMotor.calculate(data.data))
        pass 
    def windows_matlab_cmd_callback(self,data):
        pass

    def BLDC0_current_reciever_callback(self,data):
        pass
    def BLDC1_current_reciever_callback(self,data):
        pass

    

def main():
    ros_env = ROS_env()
    ros_env.mainloop()

if __name__ == '__main__':
    main()
