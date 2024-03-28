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
# from act_pump.srv import service1

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
        self.near_cs = {}
        self.threshold = 0.1
        self.good_width_ang = []

    # GetStalk is the .srv file
    # get_stalk: name of the service being called
    # stalk: service client object
    # output_1: this has three outputs - string(success), int(num_frames), stalk_detect/grasp_point[] (grasp_points) {grasp_points is an array of type stalk_detect (which is basically x,y,z coordinate)}
    # near_cs: unordered list (dict) of nearby cornstalks with key = hashvalue of grasppoints
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
                            self.near_cs[grasp_coordinates] = grasp_coordinates
                            print(f"The list is: {self.near_cs}")
                            # rospy.loginfo(f"Grasp Point {i}: x={point.position.x}, y={point.position.y}, z={point.position.z} in else")
                            return "SUCCESS"
                    else:
                        self.near_cs[grasp_coordinates] = grasp_coordinates
                        print("near_cs is empty, so first grasp point added to the list")
                        return "SUCCESS"
                    
            elif (flag == "ERROR"):
                return "ERROR"
            
            # else:
            #     return "REPOSITION"

        except rospy.ServiceException as exc:
            rospy.logerr('Service did not process request: ' + str(exc))

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
        
        return 0

    # def m_3 (self, pumps):

    #     rospy.loginfo('Go to Clean Nozzle')
    #     rospy.wait_for_service('control_pumps')
    #     xarm_status = rospy.ServiceProxy('control_pumps', service1)
    #     try:
    #         output_3 = xarm_status(pumps)
    #         status = output_3.success

    #     except rospy.ServiceException as exc:
    #         rospy.loginfo('Service did not process request: ' + str(exc))
    #     return status

# State 1 - Finding Cornstalk
#TODO: error handling. Proceed to grasp_stalk only after success flag
class state1(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['cleaning_calibrating'],
                            input_keys = ['state_1_input'])
        self.utils = utils
        self.width_ang = []
        
    def execute(self, userdata):
        # rospy.loginfo('Running State 1: Finding Cornstalk')

        # xarm_flag = stow_position()
    
        # grasp_flag = self.utils.get_grasp(userdata.state_1_input[0], userdata.state_1_input[1])
        # print("Output from find_stalk method: %s", grasp_flag)
        
        for i in range(5):
            #calling arc_move ---> returns angle
            angle = 15*i
            width = self.utils.get_width(userdata.state_1_input[0], userdata.state_1_input[1])
            pair = (angle, width)
            self.width_ang.append(pair)
        print(f"angle width pair is: {self.width_ang}")
        max_width = max(self.width_ang, key = lambda x:x[1])
        print(f"Maximum width {max_width[1]} is at angle {max_width[0]}")

        # rospy.INFO('Output from find_width method: %s', flag_2)
        # print("Output from find_stalk method: %s", flag_2)
        return 'cleaning_calibrating'

# State 2 - Cleaning and Calibrating
class state2(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['insertion','replace'],
                            input_keys = ['state_2_ip'])

    def execute(self, userdata):
        # rospy.loginfo('Running State 2: Cleaning and Calibrating')

        return 'insertion'

# State 3: Insertion
class state3(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['replace'],
                            input_keys = ['state_3_ip'])

    def execute(self, userdata):
        # rospy.loginfo('Running State 3: Insertion')

        # flag = m_3(self, userdata.state_3_ip)
        # print("Output from EM method: %s", flag)
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

class FSM:

    def __init__(self):
        rospy.init_node('nimo_state_machine')
        self.utils = Utils()
        self.main()

    def main(self):

        start_state = smach.StateMachine(outcomes = ['stop'])    # Outcome of Main State Machine
        start_state.userdata.flag_a = 1
        start_state.userdata.flag_b = 2
        start_state.userdata.flag_c = 3
        start_state.userdata.find_stalk = (2, 10.0)  # a tuple of num_frames and timeout
        start_state.userdata.gopump = "pumpsoff"

        with start_state:

            smach.StateMachine.add('Finding_Cornstalk',state1(self.utils),
                                transitions = {'cleaning_calibrating':'Cleaning_Calibrating'},
                                remapping = {'state_1_input':'find_stalk'})  # Go to State B
            
            smach.StateMachine.add('Cleaning_Calibrating',state2(self.utils),
                                transitions = {'insertion':'Insertion','replace':'stop'},
                                remapping = {'c_c_ip':'flag_b'})  # Go to State B
            
            smach.StateMachine.add('Insertion',state3(self.utils),
                                transitions = {'replace':'Replace'},
                                remapping = {'state_3_ip':'gopump'})  # Go to State B
            
            smach.StateMachine.add('Replace',state4(self.utils),
                    transitions = {'replace_stop':'Finding_Cornstalk'},    # should be 'stop' instead of 'Finding_Cornstalk'
                    remapping = {'state_4_ip':'gopump'})  # Go to State B
            
            # ADD state 4 stupid
        
        sis = smach_ros.IntrospectionServer('server_name', start_state, '/NiMo_SM')
        sis.start()

        outcome = start_state.execute()

        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    fsm = FSM()