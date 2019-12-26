#!/usr/bin/env python
'''Demo tesing programme'''
import rospy
from std_msgs.msg import Float32
import time
import math
import numpy
import collections
from my_lib import speed_series_update
from my_lib import speed_filter
from my_lib import BLDC_current_protect
from my_lib import Angle_Follow_protect
from my_lib import loadMotor_voltage
from my_lib import clutch_state_protect
from my_lib import Limitation

# Global Variables Definition
global BLDC0_current
global BLDC1_current
global rackforce
global steerwheel_angle
global steerwheel_angle_time
global steerwheel_angle_speed
global steerwheel_angle_speed_series
global roadwheel_angle
global roadwheel_angle_time
global roadwheel_angle_speed_series
global roadwheel_angle_speed
global control_frequency
global starting_time
global controls
global matlab_cmd_repo
global matlab_cmd_reception

# Global Variables Initializaion
BLDC0_current = 0    # BLDC near rack
BLDC1_current = 0    # BLDC near steering wheel
rackforce = 0    # rack force detected by the sensor, kN. Stretching: >0; Compressing: <0.
steerwheel_angle = 0    # column angle near steering wheel
roadwheel_angle = 0    # column angle near rack
steerwheel_angle_time = 0
roadwheel_angle_time = 0
steerwheel_angle_speed = 0
roadwheel_angle_speed = 0
steerwheel_angle_speed_series = [0, 0, 0, 0, 0]
roadwheel_angle_speed_series = [0, 0, 0, 0, 0]
control_frequency = 100   #   This value can not be zero or below!
starting_time = time.time()
matlab_cmd_repo = 30
matlab_cmd_reception = False

class Control_Options():

    def __init__(self):
        self.BLDC0_current = 0.0    #    BLDC0_current: unit: A.
        self.BLDC1_current = 0.0    #    BLDC1_current: unit: A.
        self.clutch_state = 0    #    clutch_state: 0 -> engaged;   1 -> disengaged.
        self.loadmotor_targetforce = 0.0    #    loadmotor_targetforce: unit: kN
        self.ctrl_quit = 0    #    ctrl_quit: 0 -> dont quit;   1 -> quit.
        self.steerwheelMotor_PID_preserved_info = [0.0, 0.0]    # [last margin, I result]
        self.roadwheelMotor_PID_preserved_info = [0.0, 0.0]    # [last margin, I result]
        self.loadMotor_PID_preserved_info = [0.0, 0.0]    # [last margin, I result]

    def steerwheelMotor_PID(self, margin, Kp, Ki, Kd):
        '''this function provides a calculation of PI controller.'''
        global control_frequency
        # calculate the P. I and D seperately
        P_result = Kp * margin
        I_result = Limitation(self.steerwheelMotor_PID_preserved_info[1] + Ki * margin / control_frequency, 1.7)
        D_result = Kd * (margin - self.steerwheelMotor_PID_preserved_info[0]) / control_frequency
        # update the preserved information
        self.steerwheelMotor_PID_preserved_info[0] = margin
        self.steerwheelMotor_PID_preserved_info[1] = I_result
        # calculate the PID output
        PI_output = P_result + I_result + D_result
        return PI_output

    def roadwheelMotor_PID(self, margin, Kp, Ki, Kd):
        '''this function provides a calculation of PI controller.'''
        global control_frequency
        # calculate the P. I and D seperately
        P_result = Kp * margin
        I_result = Limitation(self.roadwheelMotor_PID_preserved_info[1] + Ki * margin / control_frequency, 1.7)
        D_result = Kd * (margin - self.roadwheelMotor_PID_preserved_info[0]) / control_frequency
        # update the preserved information
        self.roadwheelMotor_PID_preserved_info[0] = margin
        self.roadwheelMotor_PID_preserved_info[1] = I_result
        # calculate the PID output
        PI_output = P_result + I_result + D_result
        return PI_output

    def loadMotor_PID(self, margin, Kp, Ki, Kd):
        '''this function provides a calculation of PI controller.'''
        global control_frequency
        # calculate the P. I and D seperately
        P_result = Kp * margin
        I_result = Limitation(self.loadMotor_PID_preserved_info[1] + Ki * margin / control_frequency, 6)
        D_result = Kd * (margin - self.loadMotor_PID_preserved_info[0]) / control_frequency
        # update the preserved information
        self.loadMotor_PID_preserved_info[0] = margin
        self.loadMotor_PID_preserved_info[1] = I_result
        # calculate the PID output
        PI_output = P_result + I_result + D_result
        return PI_output
    
    def stop(self):
        self.ctrl_quit = 1

controls = Control_Options()
#
###################### Signal Recieving Callbacks ################################################################

###  Here presents four callbacks, including:                      ###
###     roadwheel_angle_reciever_callback()                        ###
###     steerwheel_angle_reciever_callback()                       ###
###     BLDC0_current_reciever_callback()                          ###
###     BLDC1_current_reciever_callback()                          ###
###  Notice that these callbacks have been integrated in the       ###
###   initialization function, so no need to seperately use them.  ###
###                       DON'T ALTER!!!!                          ###

def roadwheel_angle_reciever_callback(data):
    '''Roadwheel Angle Callback Function'''
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global roadwheel_angle
    global roadwheel_angle_time
    global roadwheel_angle_speed
    global roadwheel_angle_speed_series

    roadwheel_angle_temp = data.data
    roadwheel_angle_time_temp = time.time()
    delta_time = roadwheel_angle_time_temp - roadwheel_angle_time
    delta_angle = roadwheel_angle_temp - roadwheel_angle
    if (delta_time > 0):
        roadwheel_angle_speed_temp = delta_angle / delta_time
    else:
        pass
    roadwheel_angle = roadwheel_angle_temp
    roadwheel_angle_time = roadwheel_angle_time_temp
    roadwheel_angle_speed_series = speed_series_update(roadwheel_angle_speed_series, roadwheel_angle_speed_temp)
    roadwheel_angle_speed = speed_filter(roadwheel_angle_speed_series)


def steerwheel_angle_reciever_callback(data):
    '''Steerwheel Angle Callback Function'''
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global steerwheel_angle
    global steerwheel_angle_time
    global steerwheel_angle_speed
    global steerwheel_angle_speed_series

    steerwheel_angle_temp = data.data
    steerwheel_angle_time_temp = time.time()
    delta_time = steerwheel_angle_time_temp - steerwheel_angle_time
    delta_angle = steerwheel_angle_temp - steerwheel_angle
    if (delta_time > 0):
        steerwheel_angle_speed_temp = delta_angle / delta_time
    else:
        pass
    steerwheel_angle = steerwheel_angle_temp
    steerwheel_angle_time = steerwheel_angle_time_temp
    steerwheel_angle_speed_series = speed_series_update(steerwheel_angle_speed_series, steerwheel_angle_speed_temp)
    steerwheel_angle_speed = speed_filter(steerwheel_angle_speed_series)


def BLDC0_current_reciever_callback(data):
    '''roadwheel BLDC current Callback Function'''
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global BLDC0_current

    BLDC0_current = data.data


def BLDC1_current_reciever_callback(data):
    '''roadwheel BLDC current Callback Function'''
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global BLDC1_current

    BLDC1_current = data.data

def rackforce_reciever_callback(data):
    '''roadwheel BLDC current Callback Function'''
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global rackforce

    rackforce = data.data * 2

def windows_matlab_cmd_callback(data):
    '''roadwheel BLDC current Callback Function'''
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global matlab_cmd_repo
    global matlab_cmd_reception

    matlab_cmd_repo = data.data
    matlab_cmd_reception = True


#
###################### Main Function ###########################################################################

###  Here presents the main function.                                 ###
###  The main function is a box which all functions need to fit in.   ###
###                       DON'T ALTER!!!!                             ###

def Main():
    '''The main function'''

    global steerwheel_angle
    global steerwheel_angle_time
    global steerwheel_angle_speed
    global roadwheel_angle
    global roadwheel_angle_time
    global roadwheel_angle_speed
    global control_frequency
    global BLDC0_current
    global BLDC1_current
    global rackforce
    global controls
    
    rospy.init_node('test', anonymous=True)

    rospy.Subscriber("roadWheelAngle", Float32, roadwheel_angle_reciever_callback)
    rospy.Subscriber("steerWheelAngle", Float32, steerwheel_angle_reciever_callback)
    rospy.Subscriber("BLDC0_current_feedback", Float32, BLDC0_current_reciever_callback)
    rospy.Subscriber("BLDC1_current_feedback", Float32, BLDC1_current_reciever_callback)
    rospy.Subscriber("rackforce_feedback", Float32, rackforce_reciever_callback)
    rospy.Subscriber("windows_matlab_cmd", Float32, windows_matlab_cmd_callback)

    loadMotor_pub = rospy.Publisher('loadMotor', Float32, queue_size=10)
    clutch_pub = rospy.Publisher('clutch', Float32, queue_size=10)
    BLDC0_current_pub = rospy.Publisher('BLDC0_current', Float32, queue_size=10)
    BLDC1_current_pub = rospy.Publisher('BLDC1_current', Float32, queue_size=10)
    steerwheelspeed_pub = rospy.Publisher('steerwheel_speed', Float32, queue_size=10)
    roadwheelspeed_pub = rospy.Publisher('roadwheel_speed', Float32, queue_size=10)
    windows_matlab_response_pub = rospy.Publisher('windows_matlab_response', Float32, queue_size=10)

    print('''
    Which demo do you want to enter?
    ''')
    for key, value in demo_dic.items():
        print('        ' + key + '. ' + value[1])
    user_selection = input('''
    input a letter to enter selected demo, or input other letters to quit.
    
    Please enter your selection:  ''')
    # inputcheck
    if user_selection not in demo_dic.keys():
        print('''
        Please press Ctrl + C to terminate the ROS core.
        Thank you. Have fun.
        ''')
        quit()

    # spin() simply keeps python from exiting until this node is stopped
    rate = rospy.Rate(control_frequency) # 100hz-
    starting_time = time.time()

    while (not rospy.is_shutdown()) and (controls.ctrl_quit == 0):
        roadwheelspeed_pub.publish(roadwheel_angle_speed)
        steerwheelspeed_pub.publish(steerwheel_angle_speed)
        
        demo_dic[user_selection][0]()    # User defined function
        # system_disable()    # to completely disable the system
        BLDC0_current_pub.publish(BLDC_current_protect(controls.BLDC0_current))
        BLDC1_current_pub.publish(BLDC_current_protect(-controls.BLDC1_current)) # Note that the direction of this motor is opposite to normal one.
        clutch_pub.publish(clutch_state_protect(controls.clutch_state))
        loadMotor_pub.publish(loadMotor_voltage(controls.loadmotor_targetforce))
        windows_matlab_response_pub.publish(1)

        rate.sleep()
        # rospy.spin()
    system_disable()
    print('''
        Please press Ctrl + C to terminate the ROS core.
        Thank you. Have fun.
        ''')




#
###################### Demo functions ##########################################################################

###  Last modification author: Li Mingcong                         ###
###  Last modification time: 2019.6.14                             ###
###  Last modification Note:                                       ###

###  Here presents five demo functions, including:                 ###
###     system_disable()                                           ###
###     sine_turning_demo()                                        ###
###     angle_following_demo()                                     ###
###     recover()                                                  ###
###     developer_mode()                                           ###
###  This is where you do with everything. Mind that only the      ###
###   'controls' of the class Control_Options is to be changed.    ###
###  Please change nothing else.                                   ###
###  And remember to update the 'demo_dic' and the 'name_dic'      ###
###   variables when adding new demos, or you won't be able to     ###
###   enter the new demos.                                         ###

def system_disable():
    global controls
    print(controls.BLDC0_current)
    controls.BLDC0_current = 0    # loadmotor_targetforce
    controls.BLDC1_current = 0    # loadmotor_targetforce
    controls.clutch_state = 0    # loadmotor_targetforce
    controls.loadmotor_targetforce = 0    # loadmotor_targetforce

def lowermotor_sine_turning_demo():
    global controls
    global roadwheel_angle

    angle_target = 100*math.sin(math.radians(20*(time.time())))
    print(angle_target)
    controls.BLDC0_current = controls.roadwheelMotor_PID(angle_target-roadwheel_angle, 0.25, 8, 0)
    controls.BLDC1_current = 0
    controls.clutch_state = 0
    controls.loadmotor_targetforce = 0
def uppermotor_sine_turning_demo():

    global controls
    global roadwheel_angle

    angle_target = 100*math.sin(math.radians(20*(time.time())))
    print(angle_target)
    controls.BLDC0_current = 0
    controls.BLDC1_current = controls.steerwheelMotor_PID(angle_target-steerwheel_angle, 0.25, 8, 0)
    controls.clutch_state = 0
    controls.loadmotor_targetforce = 0

def angle_following_demo():
    global controls
    global roadwheel_angle
    global steerwheel_angle

    angle_target = Angle_Follow_protect(steerwheel_angle)
    print(angle_target)
    controls.BLDC0_current = controls.roadwheelMotor_PID(angle_target-roadwheel_angle, 0.25, 8, 0)
    controls.BLDC1_current = 0
    controls.clutch_state = 1
    controls.loadmotor_targetforce = 0

def recover():
    '''This is a recovery demo used to recover everything back to initialized status.
	The clutch will desengage and the road wheel will be reset to zero position,
	 and the driver need to turn the handwheel slowly to synchronize the zero position.
	 If the angles match, the clutch will automatically engage,
	 and every other component is going to return to initialization status.'''
    global controls
    global roadwheel_angle
    global steerwheel_angle

    angle_target = 0
    margin = steerwheel_angle - roadwheel_angle
    controls.BLDC0_current = controls.roadwheelMotor_PID(angle_target-roadwheel_angle, 0.25, 8, 0)
    controls.BLDC1_current = 0
    controls.clutch_state = 1
    controls.loadmotor_targetforce = 0
    if (margin < 0.05) and (margin > -0.05) and (starting_time < time.time()-0.1):
        controls.stop()
        system_disable()
        print(steerwheel_angle)
        print(roadwheel_angle)
    print(margin)

def matlab_connection_test():
    '''This is a example of windows matlab connection, which brings support to matlab
    ros joint simulation.'''

    global controls
    global roadwheel_angle
    global matlab_cmd_repo
    global matlab_cmd_reception

    angle_target = matlab_cmd_reception * matlab_cmd_repo 
    print(angle_target)
    controls.BLDC0_current = 0
    controls.BLDC1_current = controls.steerwheelMotor_PID(angle_target-steerwheel_angle, 0.25, 8, 0)
    controls.clutch_state = 0
    controls.loadmotor_targetforce = 0

    
def developer_mode():

    global controls
    global roadwheel_angle
    global matlab_cmd_repo
    global matlab_cmd_reception

    if time.time() > starting_time + 5:
        matlab_cmd_reception = True

    angle_target = matlab_cmd_reception * matlab_cmd_repo 
    print(angle_target)
    controls.BLDC0_current = 0
    controls.BLDC1_current = controls.steerwheelMotor_PID(angle_target-steerwheel_angle, 0.25, 8, 0)
    controls.clutch_state = 0
    controls.loadmotor_targetforce = 0
    # Data collected: 1:1.1218:-5.65     2:1.8214:-7.9     3:2.49:-7.2749
    
demo_dic = collections.OrderedDict()

###  Here stores the demos written.                                ###
###  Everytime you add any new demos, please add a new line here   ###
###   with a new letter asigned to the demo for user to enter.     ###
demo_dic['a'] = [lowermotor_sine_turning_demo, 'lowermotor_sine_turning_demo']
demo_dic['b'] = [uppermotor_sine_turning_demo, 'uppermotor_sine_turning_demo']
demo_dic['c'] = [angle_following_demo, 'angle following demo']
demo_dic['d'] = [recover, 'recover']
demo_dic['e'] = [matlab_connection_test, 'matlab_connection_test']
demo_dic['f'] = [developer_mode, 'developer mode']


#
########################################### Main ################################################################

if __name__ == '__main__':
    Main()