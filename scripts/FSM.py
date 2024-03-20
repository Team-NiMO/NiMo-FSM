
'''
CLEAN THE FUCKING CODE STUPID GIRL!!!!!!!
'''

import os
import sys
import time
import math
import argparse
import signal

# ROS
import rospy
import tf2_ros
import geometry_msgs.msg

# FSM
import smach
import smach_ros

# custom helper library
# import xArm_Motion as xArm_Motion
# import utils_plot as fsm_plot

# Perception
from stalk_detect.srv import GetStalk
from stalk_detect.srv import GetWidth
from stalk_detect.msg import grasp_point

# External Mechanisms
from act_pump.srv import service1

# Global terms:
# xArm = xArm_Motion.xArm_Motion("192.168.1.196") # xArm6 IP address
# xArm.initialize_robot()

# xArm = xArm_Motion.xArm_Motion("192.168.1.196") # xArm6 IP address
# xArm.initialize_robot()

"""
UPDATE THIS
"""
sensor = False

# def m_1 (self, num_frames, timeout):
#     rospy.loginfo('Finding nearest Cornstalk')

#     rospy.wait_for_service('get_stalk')
#     # GetStalk is the .srv file
#     # get_stalk: name of the service being called
#     # stalk: service client object
#     # output_1: this has three outputs - string(success), int(num_frames), stalk_detect/grasp_point[] (grasp_points) {grasp_points is an array of type stalk_detect (which is basically x,y,z coordinate)}
#     # TODO: (should be accessible from all the methods of the class) near_cs: unordered list (dict) of nearby cornstalks with key = hashvalue of the 
#     stalk = rospy.ServiceProxy('get_stalk', GetStalk)
#     try:
#         output_1 = stalk(num_frames=num_frames, timeout=timeout)
#         grasp_points = output_1.grasp_points
#         flag = output_1.success

#         for i,point in enumerate(grasp_points):
#             rospy.loginfo('Point %d - x: %f, y: %f, z: %f', i, point.position.x, point.position.y, point.position.z)

#     except rospy.ServiceException as exc:
#         rospy.loginfo('Service did not process request: ' + str(exc))

#     return flag

# def m_2 (self, num_frames, timeout):
#     rospy.loginfo('Doing Width Detection')
#     rospy.wait_for_service('get_width')
#     stalk_width = rospy.ServiceProxy('get_width', GetWidth)
#     try:
#         output_2 = stalk_width(num_frames=num_frames, timeout=timeout)
#         width = output_2.width
#         flag = output_2.success

#         rospy.loginfo('width: %f', width)
#     except rospy.ServiceException as exc:
#         rospy.loginfo('Service did not process request: ' + str(exc))
    
#     return flag

def m_3 (self, pumps):
    
    rospy.loginfo('Go to Clean Nozzle')
    rospy.wait_for_service('control_pumps')
    xarm_status = rospy.ServiceProxy('control_pumps', service1)
    try:
        output_3 = xarm_status(pumps)
        status = output_3.success

    except rospy.ServiceException as exc:
        rospy.loginfo('Service did not process request: ' + str(exc))
    return status

def m_4 ():
    rospy.loginfo('Go to Calibrate Nozzle')

def m_5 ():
    rospy.loginfo('Logging Data')

def m_6 ():
    rospy.loginfo('Desired Arc Movement')

def m_7 ():
    rospy.loginfo('Insert Sensor-Actuate Linear Actuator')

def m_8 ():
    rospy.loginfo('Remove Sensor-Retract Linear Actuator')

def m_9 ():
    rospy.loginfo('Move arm to the nearest cornstalk')

def m_10 ():
    rospy.loginfo('Stow Position')

def m_11 ():
    rospy.loginfo('Replace Sensor')


# State 1 - Finding Cornstalk
class state1(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['cleaning_calibrating'],
                             input_keys = ['state_1_input'])
        
    def execute(self, userdata):
        rospy.loginfo('Running State 1')
        m_10()
        # flag = m_1(self, userdata.state_1_input[0], userdata.state_1_input[1])
        # print("Output from find_stalk method: %s", flag)
        # rospy.INFO('Output from find_stalk method: %s', flag)
        m_9()
        # flag_2 = m_2(self, userdata.state_1_input[0], userdata.state_1_input[1])
        # rospy.INFO('Output from find_width method: %s', flag_2)
        # print("Output from find_stalk method: %s", flag_2)
        return 'cleaning_calibrating'

# State 2 - Cleaning and Calibrating
class state2(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                            outcomes = ['insertion','replace'],
                            input_keys = ['c_c_ip'])
        
    def execute(self, userdata):
        rospy.loginfo('Running State 2')
        m_10()
        m_5()
        # decision_1 = m_3(userdata.c_c_ip)
        m_5()
        m_4()
        # decision_2 = m_3(self, userdata.c_c_ip)
        # if not decision_2:
        #     return 'replace'
        # else:
        return 'insertion'
    
# State 3: Insertion
class state3(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['replace'],
                             input_keys = ['state_3_ip'])
    
    def execute(self, userdata):
        rospy.loginfo('Running State 3')
        m_10()
        m_9()
        m_5()
        m_6()
        m_7()
        m_8()
        flag = m_3(self, userdata.state_3_ip)
        print("Output from EM method: %s", flag)
        return 'replace'
    
# State 4: Replace
class state4(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['replace_stop'])
    
    def execute(self, userdata):
        rospy.loginfo('Running State 3')
        m_10()
        m_9()
        m_5()
        m_6()
        m_7()
        m_8()
        # flag = m_3(self, userdata.state4_ip)
        # print("Output from EM method: %s", flag)
        return 'replace_stop'

        
def main():
    rospy.init_node('nimo_state_machine')

    start_state = smach.StateMachine(outcomes = ['stop'])    # Outcome of Main State Machine
    start_state.userdata.flag_a = 1
    start_state.userdata.flag_b = 2
    start_state.userdata.flag_c = 3
    start_state.userdata.find_stalk = (5,10.0)  # a tuple of num_frames and timeout
    start_state.userdata.gopump = "pumpsoff"

    with start_state:

        smach.StateMachine.add('Finding_Cornstalk',state1(),
                               transitions = {'cleaning_calibrating':'Cleaning_Calibrating'},
                               remapping = {'state_1_input':'find_stalk'})  # Go to State B
        
        smach.StateMachine.add('Cleaning_Calibrating',state2(),
                               transitions = {'insertion':'Insertion','replace':'stop'},
                               remapping = {'c_c_ip':'flag_b'})  # Go to State B
        
        smach.StateMachine.add('Insertion',state3(),
                               transitions = {'replace':'stop'},
                               remapping = {'state_3_ip':'gopump'})  # Go to State B
        
        # ADD state 4 stupid
    
    sis = smach_ros.IntrospectionServer('server_name', start_state, '/NiMo_SM')
    sis.start()

    outcome = start_state.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
