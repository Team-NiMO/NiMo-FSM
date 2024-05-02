#!/usr/bin/env python3
import time
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose
import smach
import smach_ros
import yaml
import rospkg

from nimo_perception.srv import *
from nimo_manipulation.srv import *
from nimo_end_effector.srv import *
from act_pump.srv import *

# TEMPORARY
# from amiga_path_planning.srv import *

class Utils:

    def __init__(self):
        # Load parameters from configuration file
        self.loadConfig()

        if self.verbose: rospy.loginfo("Starting nimo_fsm node")

        # Load services
        self.services()

        # Initialize internal variables
        self.near_cs = []
        self.threshold = 0.1
        self.insertion_ang = None

    def loadConfig(self):
        '''
        Load configuration from yaml file
        '''

        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('nimo_fsm')
        config_path = self.package_path + '/config/default.yaml'
        with open(config_path) as file:
            config = yaml.load(file, Loader=yaml.FullLoader)

        self.verbose = config["debug"]["verbose"]

        self.enable_perception = config["debug"]["enable_perception"]
        self.enable_manipulation = config["debug"]["enable_manipulation"]
        self.enable_end_effector = config["debug"]["enable_end_effector"]
        self.enable_external_mechanisms = config["debug"]["enable_external_mechanisms"]
        self.enable_navigation = config["debug"]["enable_navigation"]

    def services(self):
        # Load Perception Services
        if self.enable_perception:
            if self.verbose: rospy.loginfo("Waiting for perception services")
            try:
                rospy.wait_for_service('GetWidth', timeout=1)
                rospy.wait_for_service('Stalks', timeout=1)
                self.GetWidthService = rospy.ServiceProxy('GetWidth', GetWidth)
                self.GetStalksService = rospy.ServiceProxy('GetStalks', GetStalks)
            except Exception as e:
                rospy.logerr("Unable to load perception services")
                raise e

        # Load Manipulation Services
        if self.enable_manipulation:
            if self.verbose: rospy.loginfo("Waiting for manipulation services")
            try:
                rospy.wait_for_service('GoHome', timeout=1)
                rospy.wait_for_service('LookatCorn', timeout=1)
                rospy.wait_for_service('LookatAngle', timeout=1)
                rospy.wait_for_service('GoCorn', timeout=1)
                rospy.wait_for_service('UngoCorn', timeout=1)
                rospy.wait_for_service('ArcCorn', timeout=1)
                rospy.wait_for_service('HookCorn', timeout=1)
                rospy.wait_for_service('UnhookCorn', timeout=1)
                rospy.wait_for_service('GoEM', timeout=1)
                self.GoHomeService = rospy.ServiceProxy('GoHome', GoHome)
                self.LookatCornService = rospy.ServiceProxy('LookatCorn', LookatCorn)
                self.LookatAngleService = rospy.ServiceProxy('LookatAngle', LookatAngle)
                self.GoCornService = rospy.ServiceProxy('GoCorn', GoCorn)
                self.UngoCornService = rospy.ServiceProxy('UngoCorn', UngoCorn)
                self.ArcCornService = rospy.ServiceProxy('ArcCorn', ArcCorn)
                self.HookCornService = rospy.ServiceProxy('HookCorn', HookCorn)
                self.UnhookCornService = rospy.ServiceProxy('UnhookCorn', UnhookCorn)
                self.GoEMService = rospy.ServiceProxy('GoEM', GoEM)
            except Exception as e:
                rospy.logerr("Unable to load manipulation services")
                raise e
        
        # Load End Effector Services
        if self.enable_end_effector:
            if self.verbose: rospy.loginfo("Waiting for end effector services")
            try:
                rospy.wait_for_service('get_cal_dat', timeout=1)
                rospy.wait_for_service('act_linear', timeout=1)
                rospy.wait_for_service('get_dat', timeout=1)
                self.GetCalDatService = rospy.ServiceProxy('get_cal_dat', get_cal_dat)
                self.ActLinearService = rospy.ServiceProxy('act_linear', act_linear)
                self.GetDatService = rospy.ServiceProxy('get_dat', get_dat)
            except Exception as e:
                rospy.logerr("Unable to load end effector services")
                raise e
        
        # Load External Mechanisms Services
        if self.enable_external_mechanisms:
            if self.verbose: rospy.loginfo("Waiting for external mechanisms services")
            try:
                rospy.wait_for_service('control_pumps', timeout=1)
                self.ControlPumpsService = rospy.ServiceProxy('control_pumps', service1)
            except Exception as e:
                rospy.logerr("Unable to load external mechanisms services")
                raise e

        # Load Navigation Services
        if self.enable_navigation:
            if self.verbose: rospy.loginfo("Waiting for external mechanisms services")
            try:
                rospy.wait_for_service('amiga_planner', timeout=1)
                self.PlanningService = rospy.ServiceProxy('amiga_planner', planning_request)
            except Exception as e:
                rospy.logerr("Unable to load navigation services")
                raise e

    def get_grasp (self, num_frames, timeout):

        rospy.loginfo('Finding nearest Cornstalk')
        rospy.wait_for_service('GetStalks')
        stalk = rospy.ServiceProxy('GetStalks', GetStalks)

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

    def callback(self,idk):
        if self.enable_external_mechanisms:
            if self.verbose: rospy.loginfo("Calling ControlPumps Off")
            outcome = self.ControlPumpsService("pumpsoff")
            if outcome.success == "ERROR":
                rospy.logerr("ControlPumps Off failed")

# State 1: Navigate
class navigate(smach.State):
    '''
    Navigation State
    - Call planner service
    - Check nav_stat parameter until navigation is complete
    '''

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['success','error'])
        
        self.utils = utils
    
    def execute(self, userdata):
        if self.utils.verbose: rospy.loginfo("----- Entering Navigation State -----")

        # TEMPORARY - Load Next Waypoint
        pose = Pose()
        pose.position.x = 2.87
        pose.position.y = 18.47
        pose.position.z = 0

        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1

        # TODO: STOW ARM

        if self.utils.enable_navigation:
            # Call Planner
            if self.utils.verbose: rospy.loginfo("Calling planner service")
            outcome = self.utils.PlanningService(pose)

            if not outcome:
                rospy.logerr("Planner failed")
                return 'error'

            # Wait for navigation to complete
            if self.utils.verbose: rospy.loginfo("Waiting for navigation to complete...")
            while not rospy.get_param('/nav_stat'): pass

        return 'success'

# State 2 - Finding Cornstalk
class find_cornstalk(smach.State):
    '''
    Find Cornstalk State
    - Bring arm to home position
    - Look for cornstalk at multiple angles
    - Move to suitable cornstalk and find maximum width
    - Bring arm to home position
    '''

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['success','error','reposition'])
        self.utils = utils
        self.width_ang = []
        
    def execute(self, userdata):
        if self.utils.verbose: rospy.loginfo("----- Entering Find Cornstalk State -----")

        if self.utils.enable_manipulation:
            # Reset the arm
            if self.utils.verbose: rospy.loginfo("Calling GoHome")
            outcome = self.utils.GoHomeService()
            if outcome.success == "ERROR":
                rospy.logerr("GoHome failed")
                return 'error'

            # Look at the cornstalk
            if self.utils.verbose: rospy.loginfo("Calling LookatCorn")
            outcome = self.utils.LookatCornService()
            if outcome.success == "ERROR":
                rospy.logerr("LookatCorn failed")
                return 'error'

            # Rotate the end effector left and right to view cornstalks
            reposition_counter = 0
            angle_list = [-30, 0, 30]
            for angle in angle_list:
                # Rotate end effector while looking at stalk
                outcome = self.utils.LookatAngleService(joint_angle=angle)
                if outcome.success == "ERROR":
                    rospy.logerr("LookAtAngle failed")
                    return 'error'
                
                # Check for cornstalks
                if self.utils.enable_perception:
                    outcome = self.utils.get_grasp(userdata.state_1_input[0], userdata.state_1_input[1])
                    if outcome == "SUCCESS":
                        break
                    elif outcome == "REPOSITION":
                        reposition_counter += 1
                    elif outcome == "ERROR":
                        rospy.logerr("GetStalks failed")
                        return 'error'
                else:
                    # TEMPORARY
                    self.utils.near_cs.append([0, -0.4, 0.6])
                    # reposition_counter += 1
                
            # If no cornstalks are found at any angle, reposition
            if reposition_counter == len(angle_list):
                if self.utils.verbose: rospy.loginfo("No cornstalks found nearby")
                return 'reposition'
            
            
            if self.utils.verbose: rospy.loginfo("Calling GoHome")
            outcome = self.utils.GoHomeService()
            if outcome.success == "ERROR":
                rospy.logerr("GoHome failed")
                return 'error'
            
            # Move to stalk for inspection
            current_stalk = Point(x = self.utils.near_cs[-1][0],
                                y = self.utils.near_cs[-1][1],
                                z = self.utils.near_cs[-1][2])

            if self.utils.verbose: rospy.loginfo("Calling GoCorn")
            outcome = self.utils.GoCornService(grasp_point = current_stalk)
            if outcome.success == "ERROR":
                rospy.logerr("GoCorn failed")
                return 'error'
            
            # Go to minimum angle
            width_ang = []
            if self.utils.verbose: rospy.loginfo("Calling ArcCorn")
            ArcMoveOutput = self.utils.ArcCornService(relative_angle=-30)
            if ArcMoveOutput.success == "ERROR":
                rospy.logerr("GoCorn failed")
                return 'error'
            
            # Get Width
            if self.utils.enable_perception:
                if self.utils.verbose: rospy.loginfo("Calling GetWidth")
                GetWidthOutput = self.utils.GetWidthService(num_frames = userdata.state_1_input[0], timeout = userdata.state_1_input[1])
                if GetWidthOutput.success == "ERROR":
                    rospy.logerr("GetWidth failed")
                    return 'error'
                
                width_ang.append((GetWidthOutput.width, ArcMoveOutput.absolute_angle))
            else:
                width_ang.append((0, ArcMoveOutput.absolute_angle))

            # Check other angles for width
            for i in range(4):
                # Go to next angle
                if self.utils.verbose: rospy.loginfo("Calling ArcCorn")
                ArcMoveOutput = self.utils.ArcCornService(relative_angle=15)
                if ArcMoveOutput.success == "ERROR":
                    rospy.logerr("GoCorn failed")
                    return 'error'
                
                # Get Width
                if self.utils.enable_perception:
                    if self.utils.verbose: rospy.loginfo("Calling GetWidth")
                    GetWidthOutput = self.utils.GetWidthService(num_frames = userdata.state_1_input[0], timeout = userdata.state_1_input[1])
                    if GetWidthOutput.success == "ERROR":
                        rospy.logerr("GetWidth failed")
                        return 'error'
                    
                    width_ang.append((GetWidthOutput.width, ArcMoveOutput.absolute_angle))
                else:
                    width_ang.append((0, ArcMoveOutput.absolute_angle))

            max_pair = max(width_ang, key = lambda x:x[0])
            self.utils.insertion_ang = max_pair[1]
            if self.utils.verbose:
                rospy.loginfo("Width Angle list is: {}".format(width_ang))
                rospy.loginfo("Maximum insertion angle is {}".format(self.utils.insertion_ang))

            # Return to 0 angle
            width_ang = []
            if self.utils.verbose: rospy.loginfo("Calling ArcCorn")
            outcome = self.utils.ArcCornService(relative_angle=-30)
            if outcome.success == "ERROR":
                rospy.logerr("GoCorn failed")
                return 'error'
            
            if self.utils.verbose: rospy.loginfo("Calling UngoCorn")
            outcome = self.utils.UngoCornService()
            if outcome.success == "ERROR":
                rospy.logerr("UngoCorn failed")

        return 'success'

# State 3 - Cleaning and Calibrating
class clean_calibrate(smach.State):
    '''
    Clean Calibrate State
    - Bring arm to home position
    - Expose sensor to cleaning solution
    - Expose sensor to low calibration solution
    - Expose sensor to high calibration solution
    - Expose sensor to cleaning solution
    - Bring arm to home position
    '''

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['success', 'error', 'replace']) # NOTE: END EFFECTOR CURRENTLY DOES NOT CHEKC FOR REPLACEMENT AT THIS STAGE
        
        self.utils = utils

    def execute(self, userdata):
        if self.utils.verbose: rospy.loginfo("----- Entering Clean Calibrate State -----")

        if self.utils.enable_manipulation:
            # Reset the arm
            if self.utils.verbose: rospy.loginfo("Calling GoHome")
            outcome = self.utils.GoHomeService()
            if outcome.success == "ERROR":
                rospy.logerr("GoHome failed")
                return 'error'
            
            # Extend the linear actuator
            if self.utils.enable_end_effector:
                if self.utils.verbose: rospy.loginfo("Calling ActLinear Extend")
                outcome = self.utils.ActLinearService("extend")
                if outcome.success == "ERROR":
                    rospy.logerr("ActLinear Extend failed")
                    return 'error'
                
            # Move the end effector to the cleaning pump
            if self.utils.verbose: rospy.loginfo("Calling GoEM Clean")
            outcome = self.utils.GoEMService("clean")
            if outcome.success == "ERROR":
                rospy.logerr("GoEM Clean failed")
                return 'error'

            # Turn on the cleaning pump
            if self.utils.enable_external_mechanisms:
                if self.utils.verbose: rospy.loginfo("Calling ControlPumps Pump1")
                outcome = self.utils.ControlPumpsService("pump1")
                if outcome.success == "ERROR":
                    rospy.logerr("ControlPumps Pump1 failed")
                    return 'error'
                
                # Wait for 15s before turning pumps off
                rospy.timer.Timer(rospy.rostime.Duration(15), self.utils.callback, oneshot=True)

            # Move the end effector to the low calibration pump
            if self.utils.verbose: rospy.loginfo("Calling GoEM Low Calibration")
            outcome = self.utils.GoEMService("cal_low")
            if outcome.success == "ERROR":
                rospy.logerr("GoEM Low Calibration failed")
                return 'error'

            # Turn on the low calibration pump
            if self.utils.enable_external_mechanisms:
                if self.utils.verbose: rospy.loginfo("Calling ControlPumps Pump2")
                outcome = self.utils.ControlPumpsService("pump2")
                if outcome.success == "ERROR":
                    rospy.logerr("ControlPumps Pump2 failed")
                    return 'error'
                
                # Wait for 15s before turning pumps off
                rospy.timer.Timer(rospy.rostime.Duration(15), self.utils.callback, oneshot=True)

                # Record reading for low calibration
                if self.utils.enable_end_effector:
                    if self.utils.verbose: rospy.loginfo("Calling GetCalDat Low Calibration")
                    outcome = self.utils.GetCalDatService("cal_low")
                    if outcome.success == "ERROR":
                        rospy.logerr("GetCalDat Low Calibration failed")
                        return 'error'
                    
            # Move the end effector to the high calibration pump
            if self.utils.verbose: rospy.loginfo("Calling GoEM High Calibration")
            outcome = self.utils.GoEMService("cal_high")
            if outcome.success == "ERROR":
                rospy.logerr("GoEM High Calibration failed")
                return 'error'

            # Turn on the high calibration pump
            if self.utils.enable_external_mechanisms:
                if self.utils.verbose: rospy.loginfo("Calling ControlPumps Pump3")
                outcome = self.utils.ControlPumpsService("pump3")
                if outcome.success == "ERROR":
                    rospy.logerr("ControlPumps Pump3 failed")
                    return 'error'
                
                # Wait for 15s before turning pumps off
                rospy.timer.Timer(rospy.rostime.Duration(15), self.utils.callback, oneshot=True)

                # Record reading for high calibration
                if self.utils.enable_end_effector:
                    if self.utils.verbose: rospy.loginfo("Calling GetCalDat High Calibration")
                    outcome = self.utils.GetCalDatService("cal_high")
                    if outcome.success == "ERROR":
                        rospy.logerr("GetCalDat High Calibration failed")
                        return 'error'
                    
            # Move the end effector to the cleaning pump
            if self.utils.verbose: rospy.loginfo("Calling GoEM Clean")
            outcome = self.utils.GoEMService("clean")
            if outcome.success == "ERROR":
                rospy.logerr("GoEM Clean failed")
                return 'error'

            # Turn on the cleaning pump
            if self.utils.enable_external_mechanisms:
                if self.utils.verbose: rospy.loginfo("Calling ControlPumps Pump1")
                outcome = self.utils.ControlPumpsService("pump1")
                if outcome.success == "ERROR":
                    rospy.logerr("ControlPumps Pump1 failed")
                    return 'error'
                
                # Wait for 15s before turning pumps off
                rospy.timer.Timer(rospy.rostime.Duration(15), self.utils.callback, oneshot=True)

            # Retract the linear actuator
            if self.utils.enable_end_effector:
                if self.utils.verbose: rospy.loginfo("Calling ActLinear Retract")
                outcome = self.utils.ActLinearService("retract")
                if outcome.success == "ERROR":
                    rospy.logerr("ActLinear Retract failed")
                    return 'error'

            # Reset the arm
            if self.utils.verbose: rospy.loginfo("Calling GoHome")
            outcome = self.utils.GoHomeService()
            if outcome.success == "ERROR":
                rospy.logerr("GoHome failed")
                return 'error'

        return 'success'

# State 4: Insertion
class insert(smach.State):
    '''
    Insert State
    - Bring arm to home position
    - Hook cornstalk
    - Insert sensor
    - Collect Data
    - Retract Sensor
    - Bring arm to home position
    '''

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['success','error'],
                            input_keys = ['state_3_ip'])
        self.utils = utils

    def execute(self, userdata):
        if self.utils.verbose: rospy.loginfo("----- Entering Insert State -----")

        if self.utils.enable_manipulation:
            # Reset the arm
            if self.utils.verbose: rospy.loginfo("Calling GoHome")
            outcome = self.utils.GoHomeService()
            if outcome.success == "ERROR":
                rospy.logerr("GoHome failed")
                return 'error'
            
            # Hook Stalk
            current_stalk = Point(x = self.utils.near_cs[-1][0],
                                  y = self.utils.near_cs[-1][1],
                                  z = self.utils.near_cs[-1][2])
            
            if self.utils.verbose: rospy.loginfo("Calling HookCorn")
            outcome = self.utils.HookCornService(grasp_point = current_stalk, insert_angle = self.utils.insertion_ang)
            if outcome.success == "ERROR":
                rospy.logerr("HookCorn failed")
                return 'error'

            if self.utils.enable_end_effector:
                # Extend the linear actuator
                if self.utils.verbose: rospy.loginfo("Calling ActLinear Extend")
                outcome = self.utils.ActLinearService("extend")
                if outcome.success == "ERROR":
                    rospy.logerr("ActLinear Extend failed")
                    return 'error'
                
                # Collect Nitrate data
                if self.utils.verbose: rospy.loginfo("Calling GetDat")
                outcome = self.utils.GetDatService()
                if outcome.success == "ERROR":
                    rospy.logerr("GetDat failed")
                    return 'replace' 

                # Retract the linear actuator
                if self.utils.verbose: rospy.loginfo("Calling ActLinear Retract")
                outcome = self.utils.ActLinearService("retract")
                if outcome.success == "ERROR":
                    rospy.logerr("ActLinear Retract failed")
                    return 'error'
            
            # Reset the arm
            if self.utils.verbose: rospy.loginfo("Calling GoHome")
            outcome = self.utils.GoHomeService()
            if outcome.success == "ERROR":
                rospy.logerr("GoHome failed")
                return 'error'
                
        return 'success'

# State 5: Replace
class replace(smach.State):
    '''
    Replace State
    - Bring arm to home position
    - Perform replacement action
    '''

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['success', 'error'])
        self.utils = utils
    
    def execute(self):
        if self.utils.verbose: rospy.loginfo("----- Entering Replace State -----")

        if self.utils.enable_manipulation:
            # Reset the arm
            if self.utils.verbose: rospy.loginfo("Calling GoHome")
            outcome = self.utils.GoHomeService()
            if outcome.success == "ERROR":
                rospy.logerr("GoHome failed")
                return 'error'

            # TODO: Call Replacement service

        return 'success'

class FSM:
    def __init__(self):
        self.utils = Utils()
        self.main()

    def main(self):
        start_state = smach.StateMachine(outcomes = ['stop'])    # Outcome of Main State Machine
        start_state.userdata.find_stalk = (3, 10.0)  # a tuple of num_frames and timeout

        with start_state:

            smach.StateMachine.add('Navigate',navigate(self.utils),
                                transitions = {'success':'Finding_Cornstalk',
                                               'error':'stop'})

            smach.StateMachine.add('Finding_Cornstalk',find_cornstalk(self.utils),
                                transitions = {'success':'Cleaning_Calibrating',
                                               'error':'stop',
                                               'reposition':'Navigate'},
                                remapping = {'state_1_input':'find_stalk'})
            
            smach.StateMachine.add('Cleaning_Calibrating',clean_calibrate(self.utils),
                                transitions = {'success':'Insertion',
                                               'error':'stop',
                                               'replace':'Replace'})
            
            smach.StateMachine.add('Insertion',insert(self.utils),
                                transitions = {'success':'Navigate',
                                               'error':'stop',
                                               'replace':'Replace'})
            
            smach.StateMachine.add('Replace',replace(self.utils),
                                transitions = {'replace_stop':'stop'})
        
        sis = smach_ros.IntrospectionServer('server_name', start_state, '/nimo_fsm')
        sis.start()
        start_state.execute()
        sis.stop()

if __name__ == '__main__':
    rospy.init_node('nimo_fsm')
    fsm = FSM()