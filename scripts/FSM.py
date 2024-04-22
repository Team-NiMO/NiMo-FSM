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

# Perception
from stalk_detect.srv import *

# External Mechanisms
from act_pump.srv import *

# Manipulation
from nimo_manipulation.srv import *

# End Effector
from nimo_end_effector.srv import *

"""
UPDATE Readme - asap
"""
class Utils:

    def __init__(self):

        self.near_cs = []
        self.threshold = 0.1
        self.insertion_ang = None

    # GetStalk is the .srv file
    # get_stalk: name of the service being called
    # stalk: service client object
    # output_1: this has three outputs - string(success), int(num_frames), stalk_detect/grasp_point[] (grasp_points) {grasp_points is an array of type stalk_detect (which is basically x,y,z coordinate)}
    # near_cs: unordered list of nearby cornstalks
    def get_grasp (self, num_frames, timeout):

        rospy.loginfo('Finding nearest Cornstalk')
        rospy.wait_for_service('GetStalks')
        stalk = rospy.ServiceProxy('GetStalks', GetStalk)

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
    
    def services(self):
        rospy.loginfo("Waiting for services...")
        rospy.wait_for_service('GetWidth')
        self.GetWidthService = rospy.ServiceProxy('GetWidth', GetWidth)
        rospy.wait_for_service('GoHome')
        self.GoHomeService = rospy.ServiceProxy('GoHome', GoHome)
        rospy.wait_for_service('LookatCorn')
        self.LookatCornService = rospy.ServiceProxy('LookatCorn', LookatCorn)
        rospy.wait_for_service('GoCorn')
        self.GoCornService = rospy.ServiceProxy('GoCorn', GoCorn)
        rospy.wait_for_service('ArcCorn')
        self.ArcCornService = rospy.ServiceProxy('ArcCorn', ArcCorn)
        rospy.wait_for_service('HookCorn')
        self.HookCornService = rospy.ServiceProxy('HookCorn', HookCorn)
        rospy.wait_for_service('UnhookCorn')
        self.UnhookCornService = rospy.ServiceProxy('UnhookCorn', UnhookCorn)
        rospy.wait_for_service('GoEM')
        self.GoEMService = rospy.ServiceProxy('GoEM', GoEM)
        rospy.wait_for_service('UngoCorn')
        self.UngoCornService = rospy.ServiceProxy('UngoCorn', UngoCorn)
        rospy.wait_for_service('get_cal_dat')
        self.GetCalDatService = rospy.ServiceProxy('get_cal_dat', get_cal_dat)
        rospy.wait_for_service('act_linear')
        self.ActLinearService = rospy.ServiceProxy('act_linear', act_linear)
        rospy.wait_for_service('get_dat')
        self.GetDatService = rospy.ServiceProxy('get_dat', get_dat)
        rospy.wait_for_service('control_pumps')
        self.ControlPumpsService = rospy.ServiceProxy('control_pumps', service1)
        rospy.loginfo("Done")

    def callback(self,idk):
        ControlPumpsOutput = self.ControlPumpsService("pumpsoff")
        print('pumps off')

# State 1 - Finding Cornstalk
class state1(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['cleaning_calibrating','restart','find_cornstalk'],
                            input_keys = ['state_1_input'])
        self.utils = utils
        self.width_ang = []
        
    def execute(self, userdata):
        try:
            
            service_ = self.utils
            service_.services()

            # Move xArm to Home position
            HomeOutput = service_.GoHomeService()

            # If error in moving to xArm, restart FSM
            if (HomeOutput.success == "ERROR"):
                print("Cannot move arm to home position 1")
                return 'restart'
            
            LookOutput = service_.LookatCornService()

            # If error in moving to xArm, restart FSM
            if (LookOutput.success == "ERROR"):
                print("Cannot move arm to look at corn")
                return 'restart'
            
            # Get Grasp Point (last point added to the near_cs list)
            grasp_flag = self.utils.get_grasp(userdata.state_1_input[0], userdata.state_1_input[1])
            if (grasp_flag == "ERROR"):
                print("Perception Failed")
                return 'restart'
            elif (grasp_flag == "REPOSITION"):
                print("No cornstalks nearby")

                # Move xArm to Home position
                HomeOutput = service_.GoHomeService()

                # If error in moving to xArm, restart FSM
                if (HomeOutput.success == "ERROR"):
                    print("Cannot move arm to home position 2")
                    return 'restart'

                return 'restart'
            
            current_stalk = Point(x = self.utils.near_cs[-1][0],
                                  y = self.utils.near_cs[-1][1],
                                  z = self.utils.near_cs[-1][2])
            
            # Move xArm to Home position
            HomeOutput = service_.GoHomeService()
            
            # If error in moving to xArm, restart FSM
            if (HomeOutput.success == "ERROR"):
                print("Cannot move arm to home position 2")
                return 'restart'
            
            # Move xArm to that CornStalk
            Go2CornOutput = service_.GoCornService(grasp_point = current_stalk)
            if (Go2CornOutput.success == "ERROR"):
                print("Cannot move to the corn")
                return 'restart'
            
            # Find Suitable width of the Cornstalk
            width_ang = []

            ArcMoveOutput = service_.ArcCornService(relative_angle=-30)
            if (ArcMoveOutput.success == "ERROR"):
                print("Cannot perform Arc Movement")
                return 'restart'
            
            GetWidthOutput = service_.GetWidthService(num_frames = userdata.state_1_input[0], timeout = userdata.state_1_input[1])
            if (GetWidthOutput.success == "REPOSITION"):
                print("Cannot find cornstalks with suitable width")
                return 'find_cornstalk'
            # width = self.utils.get_width(userdata.state_1_input[0], userdata.state_1_input[1])
            width_ang.append((GetWidthOutput.width, ArcMoveOutput.absolute_angle))

            for i in range(4):
                ArcMoveOutput = service_.ArcCornService(relative_angle=15)
                if (ArcMoveOutput.success == "ERROR"):
                    print("Error in Arc Move")
                    return 'restart'

                GetWidthOutput = service_.GetWidthService(num_frames = userdata.state_1_input[0], timeout = userdata.state_1_input[1])
                width_ang.append((GetWidthOutput.width, ArcMoveOutput.absolute_angle))
        
            max_pair = max(width_ang, key = lambda x:x[0])
            self.utils.insertion_ang = max_pair[1]

            rospy.logwarn(width_ang)

            ArcMoveOutput = service_.ArcCornService(relative_angle=-30)
            if (ArcMoveOutput.success == "ERROR"):
                print("Cannot perform Arc Movement")
                return 'restart'
            
            UngoCornOutput = service_.UngoCornService()
            if (UngoCornOutput.success == "ERROR"):
                return 'restart'

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
                            outcomes = ['insertion','replace','restart'])
        
        self.utils = utils

    def execute(self, userdata):
        try:
            # Function Calls
            service_ = self.utils
            service_.services()

            # Move xArm to Home position
            HomeOutput = service_.GoHomeService()

            # If error in moving to xArm, restart FSM
            if (HomeOutput.success == "ERROR"):
                print("Cannot move arm to home position")
                return 'restart'
            
            # Bring out the Nitrate sensor
            ActLinearOutput = service_.ActLinearService("extend")
            if (ActLinearOutput.flag == "ERROR"):
                return 'restart'
            
            # Go to EM: clean
            GoEMOutput = service_.GoEMService("clean")
            if (GoEMOutput.success == "DONE"):
                # act pump: clean
                # time.sleep(15)
                print('clean pump')
                ControlPumpsOutput = service_.ControlPumpsService("pump1")
                if (ControlPumpsOutput.success == True):
                    rospy.timer.Timer(rospy.rostime.Duration(15), service_.callback, oneshot=True)
                    time.sleep(15)   # delay of 15 seconds - do it in a better way
                    # ControlPumpsOutput = service_.ControlPumpsService("pumpsoff")
                
            # Go to EM: calib_low
            if (ControlPumpsOutput.success == True):
                GoEMOutput = service_.GoEMService("cal_low")
            
            if (GoEMOutput.success == "DONE"):
                # act pump: calib_low
                # time.sleep(15)
                print('cal low pump')
                ControlPumpsOutput = service_.ControlPumpsService("pump2")
                if (ControlPumpsOutput.success == True):

                    # Get Cal data
                    rospy.timer.Timer(rospy.rostime.Duration(15), service_.callback, oneshot=True)
                    CalDatOutput = service_.GetCalDatService("cal_low")
                    time.sleep(15)   # delay of 15 seconds - do it in a better way

            # Go to EM: calib_high
            if (ControlPumpsOutput.success == True):
                GoEMOutput = service_.GoEMService("cal_high")

            if (GoEMOutput.success == "DONE"):
                # act pump: calib_high
                # time.sleep(15)
                print('cal high pump')
                ControlPumpsOutput = service_.ControlPumpsService("pump3")
                if (ControlPumpsOutput.success == True):

                    # Get Cal data
                    rospy.timer.Timer(rospy.rostime.Duration(15), service_.callback, oneshot=True)
                    CalDatOutput1 = service_.GetCalDatService("cal_high")
                    time.sleep(15)   # delay of 15 seconds - do it in a better way
                    
            # Go to EM: clean
            if (ControlPumpsOutput.success == True):
                GoEMOutput = service_.GoEMService("clean")
            # GoEMOutput = service_.GoEMService("clean")

            if (GoEMOutput.success == "DONE"):
                # act pump: clean
                # time.sleep(15)
                print('clean pump')
                ControlPumpsOutput = service_.ControlPumpsService("pump1")
                if (ControlPumpsOutput.success == True):
                    rospy.timer.Timer(rospy.rostime.Duration(15), service_.callback, oneshot=True)
                    time.sleep(15)   # delay of 15 seconds - do it in a better way
                    
            if (CalDatOutput1.flag == "ERROR"):
                return 'replace'
            
            # Put the Nitrate Sensor back in
            ActLinearOutput1 = service_.ActLinearService("retract")
            if (ActLinearOutput1.flag == "ERROR"):
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
            # Function Calls
            service_ = self.utils
            service_.services()

            # Move xArm to Home position
            HomeOutput = service_.GoHomeService()

            # If error in moving to xArm, restart FSM
            if (HomeOutput.success == "ERROR"):
                print("Cannot move arm to home position insert")
                return 'restart'

            current_stalk = Point(x = self.utils.near_cs[-1][0],
                                  y = self.utils.near_cs[-1][1],
                                  z = self.utils.near_cs[-1][2])
            
            GoHook = service_.HookCornService(grasp_point = current_stalk, insert_angle = self.utils.insertion_ang)
            if (GoHook.success == "ERROR"):
                print("Error in Hooking")
                return 'restart'
            
            ActLinearOutput = service_.ActLinearService("extend")
            if (ActLinearOutput.flag == "SUCCESS"):
                GetDatOutput = service_.GetDatService()
            
            rospy.logwarn(GetDatOutput.nitrate_val)

            ActLinearOutput1 = service_.ActLinearService("retract")

            Unhook = service_.UnhookCornService()
            if (Unhook.success == "ERROR"):
                print("Error in Unhooking")
                return 'restart'

            if (GetDatOutput.flag == "ERROR"):
                print("Replace Sensor")
                return 'replace'
            
            return 'restart'
            
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            return 'restart'

        return 'restart'

# State 4: Replace/Stop
class state4(smach.State):

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['replace_stop'])
    
    def execute(self, userdata):


        return 'replace_stop'

class FSM:

    def __init__(self):
        rospy.init_node('nimo_state_machine')
        self.utils = Utils()
        self.main()

    def main(self):

        start_state = smach.StateMachine(outcomes = ['stop'])    # Outcome of Main State Machine
        start_state.userdata.find_stalk = (3, 10.0)  # a tuple of num_frames and timeout

        with start_state:

            smach.StateMachine.add('Finding_Cornstalk',state1(self.utils),
                                transitions = {'cleaning_calibrating':'Cleaning_Calibrating', # NOTE: TEMPORARY CHANGE BACK
                                               'restart':'stop',
                                               'find_cornstalk':'Finding_Cornstalk'},
                                remapping = {'state_1_input':'find_stalk'})  # Go to State B
            
            smach.StateMachine.add('Cleaning_Calibrating',state2(self.utils),
                                transitions = {'insertion':'Insertion','replace':'Replace', 'restart':'stop'})  # Go to State B
            
            smach.StateMachine.add('Insertion',state3(self.utils),
                                transitions = {'replace':'Replace', 'restart':'Finding_Cornstalk'})  # Go to State C
            
            smach.StateMachine.add('Replace',state4(self.utils),
                                transitions = {'replace_stop':'stop'})  # Go to State B
        
        
        sis = smach_ros.IntrospectionServer('server_name', start_state, '/NiMo_SM')
        sis.start()

        outcome = start_state.execute()

        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    fsm = FSM()