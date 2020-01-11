#!/usr/bin/env python
'''simulation_env ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from BLDC_model import BLDC

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
    BLDC1.force_count = BLDC1.force_count + 1
    if BLDC1.force_count >= 100:
        print("force: ",data.data)
        BLDC1.force_count = 0
    BLDC1.force = data.data


def main():
    rospy.init_node('simulation_train_env', anonymous=True)
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
            # print(BLDC1.force)

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
