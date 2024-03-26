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
UPDATE Readme - asap
"""
class Utils:

    def __init__(self):
        print("Utils class")
        self.near_cs = {}

    # GetStalk is the .srv file
    # get_stalk: name of the service being called
    # stalk: service client object
    # output_1: this has three outputs - string(success), int(num_frames), stalk_detect/grasp_point[] (grasp_points) {grasp_points is an array of type stalk_detect (which is basically x,y,z coordinate)}
    # TODO: (should be accessible from all the methods of the class) near_cs: unordered list (dict) of nearby cornstalks with key = hashvalue of grasppoints
    def get_grasp (self, num_frames, timeout):

        rospy.loginfo('Finding nearest Cornstalk')
        rospy.wait_for_service('get_stalk')
        stalk = rospy.ServiceProxy('get_stalk', GetStalk)

        try:
            print("in try")
            output_1 = stalk(num_frames=num_frames, timeout=timeout)
            flag = output_1.success
            grasp_points = output_1.grasp_points
            print(f"is this working? {flag}")

            # for i,point in enumerate(grasp_points):
            #     print(f"Grasp Point {i}: x={point.position.x}, y={point.position.y}, z={point.position.z}")

            # if (flag == "SUCCESS"):

            #     for i,point in enumerate(grasp_points):
            #         grasp_coordinates = (point.position.x, point.position.y, point.position.z)
            #         print("Print the grasp_coordinates: x: %f, y: %f, z: %f", point.position.x, point.position.y, point.position.z)
            #         if grasp_coordinates in self.near_cs:
            #             print("Print the grasp_coordinates: x: %f, y: %f, z: %f", point.position.x, point.position.y, point.position.z)
            #             print("Print the near_cs list")
            #             print(self.near_cs)
            #             print("Corn already visited")
            #             continue #if point is in the near_cs dictionary, then continue to the next grasp_point 

            #         else:
            #             self.near_cs[grasp_coordinates] = point
            #             rospy.loginfo('Point %d - x: %f, y: %f, z: %f', i, point.position.x, point.position.y, point.position.z)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        return flag
    
    # def suitable_width (self, num_frames, timeout):  # m_2 --> suitable_width
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
        
    def execute(self, userdata):
        rospy.loginfo('Running State 1: Finding Cornstalk')

        # xarm_flag = stow_position()

        grasp_flag = self.utils.get_grasp(userdata.state_1_input[0], userdata.state_1_input[1])
        print("Output from find_stalk method: %s", grasp_flag)
        
        # flag_2 = m_2(self, userdata.state_1_input[0], userdata.state_1_input[1])

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
        rospy.loginfo('Running State 2')

        return 'insertion'

# State 3: Insertion
class state3(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['replace'],
                            input_keys = ['state_3_ip'])

    def execute(self, userdata):
        rospy.loginfo('Running State 3')

        # flag = m_3(self, userdata.state_3_ip)
        # print("Output from EM method: %s", flag)
        return 'replace'

# State 4: Replace
class state4(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['replace_stop'])
    
    def execute(self, userdata):
        rospy.loginfo('Running State 3')
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
        start_state.userdata.find_stalk = (5, 10.0)  # a tuple of num_frames and timeout
        start_state.userdata.gopump = "pumpsoff"

        with start_state:

            smach.StateMachine.add('Finding_Cornstalk',state1(self.utils),
                                transitions = {'cleaning_calibrating':'Cleaning_Calibrating'},
                                remapping = {'state_1_input':'find_stalk'})  # Go to State B
            
            smach.StateMachine.add('Cleaning_Calibrating',state2(self.utils),
                                transitions = {'insertion':'Insertion','replace':'stop'},
                                remapping = {'c_c_ip':'flag_b'})  # Go to State B
            
            smach.StateMachine.add('Insertion',state3(self.utils),
                                transitions = {'replace':'stop'},
                                remapping = {'state_3_ip':'gopump'})  # Go to State B
            
            # ADD state 4 stupid
        
        sis = smach_ros.IntrospectionServer('server_name', start_state, '/NiMo_SM')
        sis.start()

        outcome = start_state.execute()

        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    fsm = FSM()
