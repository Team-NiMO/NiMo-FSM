import os
import sys
import time
import math
import argparse
import signal
import numpy as np

# ROS
import rospy
import tf2_ros
from geometry_msgs.msg import Point

# FSM
import smach
import smach_ros

# custom helper library
# import xArm_Motion as xArm_Motion
# import utils_plot as fsm_plot

# Perception
from stalk_detect.srv import GetStalk
from stalk_detect.srv import GetWidth

# External Mechanisms
# from act_pump.srv import service1

# Manipulation
from nimo_manipulation.srv import *

# Global terms:
# xArm = xArm_Motion.xArm_Motion("192.168.1.196") # xArm6 IP address
# xArm.initialize_robot()

# xArm = xArm_Motion.xArm_Motion("192.168.1.196") # xArm6 IP address
# xArm.initialize_robot()

"""
UPDATE Readme - asap
"""
class Utils:

    def __init__(self):
        #TODO: Empty the list after every restart operation?
        self.near_cs = []
        self.threshold = 0.1
        self.insertion_ang = None

        self.GoHomeService = None
        self.GoCornService = None
        self.ArcCornService = None

    # GetStalk is the .srv file
    # get_stalk: name of the service being called
    # stalk: service client object
    # output_1: this has three outputs - string(success), int(num_frames), stalk_detect/grasp_point[] (grasp_points) {grasp_points is an array of type stalk_detect (which is basically x,y,z coordinate)}
    # near_cs: unordered list of nearby cornstalks
    def get_grasp (self, num_frames, timeout):

        rospy.loginfo('Finding nearest Cornstalk')
        rospy.wait_for_service('get_stalk')
        stalk = rospy.ServiceProxy('get_stalk', GetStalk)

        try:
        
            output_1 = stalk(num_frames=num_frames, timeout=timeout)
            flag = output_1.success
            grasp_points = output_1.grasp_points

            if (flag == "DONE"):

                for i, point in enumerate(grasp_points):
                    grasp_coordinates = (point.position.x, point.position.y, point.position.z)
                    print(f"Grasp Point {i}: x={point.position.x}, y={point.position.y}, z={point.position.z}")
                    new = True

                    if bool(self.near_cs):
                        # print("if near_cs is not empty")
                        for j, cs in enumerate(self.near_cs):
                            
                            # print(f"grasp_point: {grasp_coordinates}, cs: {cs}")
                            print("Comparing {} with {} = {}".format(i, j, np.linalg.norm(np.array(grasp_coordinates) - np.array(cs))))
                            if (np.linalg.norm(np.array(grasp_coordinates) - np.array(cs))) < self.threshold:
                                # print((np.linalg.norm(np.array(grasp_coordinates) - np.array(cs))))
                                # print(f"Grasp Point {i}: x={point.position.x}, y={point.position.y}, z={point.position.z} in if")
                                # print("Print the near_cs list")
                                print(self.near_cs)
                                print("Corn already visited")
                                new = False
                                break

                        if new == True:
                            print("Grasp point not in the list, so add it to the list")
                            self.near_cs.append(grasp_coordinates)
                            print(f"The list is: {self.near_cs}")
                            # rospy.loginfo(f"Grasp Point {i}: x={point.position.x}, y={point.position.y}, z={point.position.z} in else")
                            return "SUCCESS"
                    else:
                        self.near_cs.append(grasp_coordinates)
                        print("near_cs is empty, so first grasp point added to the list")
                        return "SUCCESS"
                    
            elif (flag == "ERROR"):
                return "ERROR"

        except rospy.ServiceException as exc:
            rospy.logerr('Service did not process request: ' + str(exc))
            return "ERROR"

        return "REPOSITION"
    
    def get_width (self, num_frames, timeout):
        rospy.loginfo('Doing Width Detection')
        rospy.wait_for_service('get_width')
        stalk_width = rospy.ServiceProxy('get_width', GetWidth)
        try:
            output_2 = stalk_width(num_frames=num_frames, timeout=timeout)
            width = output_2.width
            flag = output_2.success

            if (flag == "DONE"):
                return width

            rospy.loginfo('width: %f', width)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            return "ERROR"
        
        return "ERROR"
    
    def manipulation(self):
        rospy.wait_for_service('GoHome')
        self.GoHomeService = rospy.ServiceProxy('GoHome', GoHome)
        rospy.wait_for_service('GoCorn')
        self.GoCornService = rospy.ServiceProxy('GoCorn', GoCorn)
        rospy.wait_for_service('ArcCorn')
        self.ArcCornService = rospy.ServiceProxy('ArcCorn', ArcCorn)
        rospy.wait_for_service('HookCorn')
        self.HookCornService = rospy.ServiceProxy('HookCorn', HookCorn)
        rospy.wait_for_service('UnhookCorn')
        self.UnhookCornService = rospy.ServiceProxy('UnhookCorn', UnhookCorn)


# State 1 - Finding Cornstalk
#TODO: error handling. Proceed to grasp_stalk only after success flag
class state1(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['cleaning_calibrating','restart'],
                            input_keys = ['state_1_input'])
        self.utils = utils
        self.width_ang = []
        
    def execute(self, userdata):

        try:
            
            manipulator = self.utils
            manipulator.manipulation()
            # Move xArm to Home position
            HomeOutput = manipulator.GoHomeService()
            HomeFlag = HomeOutput.success

            # If error in moving to xArm, restart FSM
            if (HomeFlag == "ERROR"):
                print("Error in HomeFlag")
                return 'restart'
            
            # Get Grasp Point (last point added to the near_cs list)
            grasp_flag = self.utils.get_grasp(userdata.state_1_input[0], userdata.state_1_input[1])
            if (grasp_flag == "ERROR"):
                print("Error in Grasp flag")
                return 'restart'
            elif (grasp_flag == "REPOSITION"):
                return 'restart'
            current_stalk = Point(x = self.utils.near_cs[-1][0],
                                  y = self.utils.near_cs[-1][1],
                                  z = self.utils.near_cs[-1][2])
            
            # Move xArm to that CornStalk
            Go2CornOutput = manipulator.GoCornService(grasp_point = current_stalk)
            if (Go2CornOutput.success == "ERROR"):
                print("Error in Go 2 corn")
                return 'restart'
            
            # Find Suitable width of the Cornstalk
            width_ang = []

            ArcMoveOutput = manipulator.ArcCornService(relative_angle=-30)
            if (ArcMoveOutput.success == "ERROR"):
                print("Error in Arc Move")
                return 'restart'
            
            width = self.utils.get_width(userdata.state_1_input[0], userdata.state_1_input[1])
            width_ang.append((width, ArcMoveOutput.absolute_angle))

            for i in range(4):
                ArcMoveOutput = manipulator.ArcCornService(relative_angle=15)
                if (ArcMoveOutput.success == "ERROR"):
                    print("Error in Arc Move")
                    return 'restart'
                width = self.utils.get_width(userdata.state_1_input[0], userdata.state_1_input[1])
                width_ang.append((width, ArcMoveOutput.absolute_angle))
        
            # print(f"angle width pair is: {self.width_ang}")
            max_pair = max(width_ang, key = lambda x:x[0])
            self.utils.insertion_ang = max_pair[1]

            print(f"Width Angle list is: {width_ang}")
            print(f"Insertion angle is: {self.utils.insertion_ang}")

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            return 'restart'

        return 'cleaning_calibrating'

# State 2 - Cleaning and Calibrating
class state2(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['insertion','replace','restart'],
                            input_keys = ['state_2_ip'])
        
        self.utils = utils

    def execute(self, userdata):
        try:
            manipulator = self.utils
            manipulator.manipulation()
            # Move xArm to Home position
            HomeOutput = manipulator.GoHomeService()
            HomeFlag = HomeOutput.success

            # If error in moving to xArm, restart FSM
            if (HomeFlag == "ERROR"):
                print("Error in HomeFlag")
                return 'restart'
            
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            return 'restart'

        return 'insertion'

# State 3: Insertion
class state3(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['replace','restart'],
                            input_keys = ['state_3_ip'])
        self.utils = utils

    def execute(self, userdata):

        try:
            manipulator = self.utils
            manipulator.manipulation()
            # Move xArm to Home position
            HomeOutput = manipulator.GoHomeService()
            HomeFlag = HomeOutput.success

            # If error in moving to xArm, restart FSM
            if (HomeFlag == "ERROR"):
                print("Error in HomeFlag")
                return 'restart'

            current_stalk = Point(x = self.utils.near_cs[-1][0],
                                  y = self.utils.near_cs[-1][1],
                                  z = self.utils.near_cs[-1][2])
                
            # # Move xArm to that CornStalk
            # Go2CornOutput = manipulator.GoCornService(grasp_point = current_stalk)
            # if (Go2CornOutput.success == "ERROR"):
            #     print("Error in Go 2 corn")
            #     return 'restart'
            
            GoHook = manipulator.HookCornService(grasp_point = current_stalk, insert_angle = self.utils.insertion_ang)
            if (GoHook.success == "ERROR"):
                print("Error in Hook")
                return 'restart'
            
            print("Extend Linear Actuator")
            print("Get Data")
            print("Retract Linear Actuator")

            Unhook = manipulator.UnhookCornService()
            if (Unhook.success == "ERROR"):
                print("Error in Unhooking")
                return 'restart'
            
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            return 'restart'

        return 'replace'

# State 4: Replace
class state4(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['replace_stop'],
                            input_keys = ['state_4_ip'])
    
    def execute(self, userdata):
        # rospy.loginfo('Running State 4')
        # flag = m_3(self, userdata.state4_ip)
        # print("Output from EM method: %s", flag)

        return 'replace_stop'
    
class state5(smach.State):

    def __init__(self,utils):
        smach.State.__init__(self,
                             outcomes = ['error'])
        
    def execute(self):
        return 'error'

class FSM:

    def __init__(self):
        rospy.init_node('nimo_state_machine')
        self.utils = Utils()
        self.main()

    def main(self):

        start_state = smach.StateMachine(outcomes = ['stop'])    # Outcome of Main State Machine
        start_state.userdata.find_stalk = (3, 10.0)  # a tuple of num_frames and timeout
        start_state.userdata.gopump = "pumpsoff"

        with start_state:

            smach.StateMachine.add('Finding_Cornstalk',state1(self.utils),
                                transitions = {'cleaning_calibrating':'Cleaning_Calibrating',
                                               'restart':'Stop'},
                                remapping = {'state_1_input':'find_stalk'})  # Go to State B
            
            smach.StateMachine.add('Cleaning_Calibrating',state2(self.utils),
                                transitions = {'insertion':'Insertion','replace':'stop', 'restart':'Stop'},
                                remapping = {'c_c_ip':'flag_b'})  # Go to State B
            
            smach.StateMachine.add('Insertion',state3(self.utils),
                                transitions = {'replace':'Replace', 'restart':'Stop'},
                                remapping = {'state_3_ip':'gopump'})  # Go to State C
            
            smach.StateMachine.add('Replace',state4(self.utils),
                                transitions = {'replace_stop':'stop'},    # should be 'stop' instead of 'Finding_Cornstalk'
                                remapping = {'state_4_ip':'gopump'})  # Go to State B
            
            smach.StateMachine.add('Stop',state5(self.utils),
                                transitions = {'error':'stop'})   # incase of any ERROR, restart from the beginning
        
        sis = smach_ros.IntrospectionServer('server_name', start_state, '/NiMo_SM')
        sis.start()

        outcome = start_state.execute()

        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    fsm = FSM()