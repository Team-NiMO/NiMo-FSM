#!/usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Bool
import smach
import smach_ros
import yaml
import rospkg
import os
import datetime
import time

from nimo_perception.srv import *
from nimo_manipulation.srv import *
from nimo_end_effector.srv import *
from act_pump.srv import *

# from amiga_path_planning.srv import *

class Utils:

    def __init__(self):
        # Load parameters from configuration file
        self.loadConfig()

        if self.verbose: rospy.loginfo("Starting nimo_fsm node for JL gripper")

        # Load services
        self.services()

        # variables (offsets seen on the field)
        self.x_offset = 0.0

        # Initialize internal variables
        self.threshold = 0.1
        self.insertion_ang = None
        self.sensor_insert_num = 0
        self.inserts = 0
        self.curr_sensor_slot = 5
        self.sensor_limit = 2  # change how frequent sensor swapping should be
        if self.enable_navigation:
            self.near_cs = [1] # Initialized so that navigation advances to waypoint instead of reposition
        else:
            self.near_cs = []

        # Setup data file
        try:
            self.run_index = max([int(f[len("RUN"):].split(".")[0]) for f in os.listdir(self.package_path+"/output")]) + 1
        except:
            self.run_index = 0

        if self.verbose: rospy.loginfo("Creating RUN{}.csv".format(self.run_index))
        f = open(self.package_path+"/output/RUN{}.csv".format(self.run_index),"w")
        f.write("time,x,y,nitrate_value\n")

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
        self.enable_arc_corn = config["debug"]["enable_arc_corn"]
        self.enable_replacement = config["debug"]["enable_replacement"]

        # Fake perception cannot be enabled if real perception is
        self.enable_fake_perception = config["debug"]["enable_fake_perception"] and not self.enable_perception
        if config["debug"]["enable_fake_perception"] and self.enable_perception:
            rospy.logwarn("Real and Fake perception cannot both be enabled, defaulting to real perception")

        self.sensor_fail_threshold = config["sensor"]["sensor_fail_threshold"]
        self.sensor_replacement = config["gripper"]["replacement"]
        self.clean_extend = config["gripper"]["clean_extend"]

    def services(self):
        # Load Perception Services
        if self.enable_perception:
            if self.verbose: rospy.loginfo("Waiting for perception services")
            try:
                rospy.wait_for_service('GetWidth', timeout=1)
                rospy.wait_for_service('GetStalks', timeout=1)
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
                rospy.wait_for_service('GoStow', timeout=1)
                rospy.wait_for_service('GoScan', timeout=1)
                rospy.wait_for_service('LookatCorn', timeout=1)
                rospy.wait_for_service('LookatAngle', timeout=1)
                rospy.wait_for_service('GoCorn', timeout=1)
                rospy.wait_for_service('UngoCorn', timeout=1)
                rospy.wait_for_service('ArcCorn', timeout=1)
                rospy.wait_for_service('HookCorn', timeout=1)
                rospy.wait_for_service('UnhookCorn', timeout=1)
                rospy.wait_for_service('GoEM', timeout=1)
                rospy.wait_for_service('GoRM', timeout=1)
                self.GoHomeService = rospy.ServiceProxy('GoHome', GoHome)
                self.GoStowService = rospy.ServiceProxy('GoStow', GoStow)
                self.GoScanService = rospy.ServiceProxy('GoScan', GoScan)
                self.LookatCornService = rospy.ServiceProxy('LookatCorn', LookatCorn)
                self.LookatAngleService = rospy.ServiceProxy('LookatAngle', LookatAngle)
                self.GoCornService = rospy.ServiceProxy('GoCorn', GoCorn)
                self.UngoCornService = rospy.ServiceProxy('UngoCorn', UngoCorn)
                self.ArcCornService = rospy.ServiceProxy('ArcCorn', ArcCorn)
                self.HookCornService = rospy.ServiceProxy('HookCorn', HookCorn)
                self.UnhookCornService = rospy.ServiceProxy('UnhookCorn', UnhookCorn)
                self.GoEMService = rospy.ServiceProxy('GoEM', GoEM)
                self.GoRMService = rospy.ServiceProxy('GoRM', GoRM)
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
                self.PlanningService = rospy.ServiceProxy('amiga_planner', planning_request_2)
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
                        for j, cs in enumerate(self.near_cs):
                            
                            print("Comparing {} with {} = {}".format(i, j, np.linalg.norm(np.array(grasp_coordinates) - np.array(cs))))
                            if (np.linalg.norm(np.array(grasp_coordinates) - np.array(cs))) < self.threshold:
                                print(self.near_cs)
                                print("Corn already visited")
                                new = False
                                break

                        if new == True:
                            print("Grasp point not in the list, so add it to the list")
                            self.near_cs.append(grasp_coordinates)
                            print(f"The list is: {self.near_cs}")
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


# State 1 - Finding Cornstalk
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
                            outcomes = ['success','error','reposition', 'next'],
                            input_keys = ['state_1_input'])
        self.utils = utils
        self.width_ang = []
        
    def execute(self, userdata):
        if self.utils.verbose: rospy.loginfo("----- Entering Find Cornstalk State -----")

        if self.utils.enable_manipulation:
            # Reset the arm
            if self.utils.verbose: rospy.loginfo("Calling GoScan")
            outcome = self.utils.GoScanService()
            if outcome.success == "ERROR":
                rospy.logerr("GoScan failed")
                return 'error'

            # Look at the cornstalk
            if self.utils.verbose: rospy.loginfo("Calling LookatCorn")
            outcome = self.utils.LookatCornService()
            if outcome.success == "ERROR":
                rospy.logerr("LookatCorn failed")
                return 'error'

            # Rotate the end effector left and right to view cornstalks
            reposition_counter = 0
            angle_list = [0, -30, 30]
            for angle in angle_list:
                # Rotate end effector while looking at stalk
                outcome = self.utils.LookatAngleService(joint_angle=angle)
                if outcome.success == "ERROR":
                    rospy.logerr("LookAtAngle failed")
                    if self.utils.verbose: rospy.loginfo("Returning to Scan Position")
                    outcome = self.utils.GoScanService()
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
                        if self.utils.verbose: rospy.loginfo("Returning to Scan Position")
                        outcome = self.utils.GoScanService()
                        return 'error'

                # Create a fake cornstalk detection if it hasn't already been done
                elif self.utils.enable_fake_perception and len(self.utils.near_cs) == 0:
                        self.utils.near_cs.append([0.1, -0.381, 0.75])
                        break
                else:
                    reposition_counter += 1
                
            # If no cornstalks are found at any angle, reposition
            if reposition_counter == len(angle_list):
                if self.utils.verbose: 
                    rospy.loginfo("No cornstalks found nearby")
                    rospy.loginfo("Returning to Scan Position")
                outcome = self.utils.GoScanService()
                return 'reposition'
            
            if self.utils.enable_arc_corn:
                if self.utils.verbose: rospy.loginfo("Calling GoScan")
                outcome = self.utils.GoScanService()
                if outcome.success == "ERROR":
                    rospy.logerr("GoScan failed")
                    return 'error'
                
                current_stalk = Point(x = self.utils.near_cs[-1][0] + self.utils.x_offset,
                                    y = self.utils.near_cs[-1][1],
                                    z = 0.76)

                if self.utils.verbose: rospy.loginfo("Calling GoCorn")
                outcome = self.utils.GoCornService(grasp_point = current_stalk)
                if outcome.success == "ERROR":
                    rospy.logerr("GoCorn failed")
                    return 'error'
                
                # Go to minimum angle
                width_ang = []
                if self.utils.verbose: rospy.loginfo("Calling ArcCorn")
                ArcMoveOutput = self.utils.ArcCornService(relative_angle=-15)
                if ArcMoveOutput.success == "ERROR":
                    rospy.logerr("GoCorn failed")
                    return 'error'
                
                # Get Width
                if self.utils.enable_perception:
                    if self.utils.verbose: rospy.loginfo("Calling GetWidth")
                    GetWidthOutput = self.utils.GetWidthService(num_frames = userdata.state_1_input[0], timeout = userdata.state_1_input[1])
                    if GetWidthOutput.success == "ERROR":
                        rospy.logerr("GetWidth failed. Moving to next angle")
                    else:
                        width_ang.append((GetWidthOutput.width, ArcMoveOutput.absolute_angle))
                else:
                    width_ang.append((0, ArcMoveOutput.absolute_angle))

                # Check other angles for width
                for i in range(2):
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
                            rospy.logerr("GetWidth failed. Moving to next angle")
                        else:
                            width_ang.append((GetWidthOutput.width, ArcMoveOutput.absolute_angle))
                    else:
                        width_ang.append((0, ArcMoveOutput.absolute_angle))

                # Return to 0 angle
                if self.utils.verbose: rospy.loginfo("Calling ArcCorn")
                outcome = self.utils.ArcCornService(relative_angle=-15)
                if outcome.success == "ERROR":
                    rospy.logerr("GoCorn failed")
                    return 'error'

                if len(width_ang) == 0:
                    rospy.logerr("GetWidth failed on every angle. Move to next stalk")
                    self.utils.insertion_ang = 0

                    # return 'next'
                else:
                    max_pair = max(width_ang, key = lambda x:x[0])
                    self.utils.insertion_ang = max_pair[1]
                if self.utils.verbose:
                    rospy.loginfo("Width Angle list is: {}".format(width_ang))
                    rospy.loginfo("Maximum insertion angle is {}".format(self.utils.insertion_ang))
            else:
                self.utils.insertion_ang = 0
            
        return 'success'

# State 2: visualServoing
class visual_servoing(smach.State):
    '''
    Visual Servoing State
    - Hook cornstalk
    - Visual servo to align to the vertical center of the stalk
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

            # Approach the cornstalk
            current_stalk = Point(x = self.utils.near_cs[-1][0],
                                  y = self.utils.near_cs[-1][1],
                                  z = 0.76)

            if self.utils.verbose: rospy.loginfo("Calling HookCorn")
            outcome = self.utils.HookCornService(grasp_point = current_stalk, insert_angle = self.utils.insertion_ang)
            if outcome.success == "ERROR":
                rospy.logerr("HookCorn failed")
                return 'error'

            # Implement Visual Servoing algorithm
        
        return 'success'

# State 4: Insertion
class insertion(smach.State):
    '''
    Insertion State
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

            if self.utils.enable_end_effector:
                # Extend the linear actuator
                if self.utils.verbose: rospy.loginfo("Calling ActLinear Extend")
                outcome = self.utils.ActLinearService("extend")
                if outcome.flag == "ERROR":
                    rospy.logerr("ActLinear Extend failed")
                    return 'error'
                
                # Collect Nitrate data
                if self.utils.verbose: rospy.loginfo("Calling GetDat")
                outcome = self.utils.GetDatService()
                if outcome.flag == "ERROR":
                    rospy.logerr("GetDat failed")
                else:
                    # Write the time, position, and nitrate value to file
                    time_str = datetime.datetime.now().strftime("%d-%m-%Y-%H:%M:%S")
                    # pose_str = "{}, {}".format(self.utils.current_pose.position.x, self.utils.current_pose.position.y)
                    f = open(self.utils.package_path+"/output/RUN{}.csv".format(self.utils.run_index), "a")
                    if self.utils.verbose: rospy.loginfo("Writing nitrate value {} PPM to RUN{}.csv".format(outcome.nitrate_val, self.utils.run_index))
                    # f.write(time_str+","+pose_str+","+"{}\n".format(120))
                    f.write(time_str+","+","+","+"{}\n".format(outcome.nitrate_val))
                    f.close()
                    
                # Retract the linear actuator
                if self.utils.verbose: rospy.loginfo("Calling ActLinear Retract")
                outcome = self.utils.ActLinearService("retract")
                if outcome.flag == "ERROR":
                    rospy.logerr("ActLinear Retract failed")
                    return 'error'

                self.utils.inserts += 1
                
                
            # Reset the arm
            if self.utils.verbose: rospy.loginfo("Calling Unhook")
            outcome = self.utils.UnhookCornService()
            if outcome.success == "ERROR":
                rospy.logerr("Unhook failed")
                return 'error'
            
            if self.utils.verbose: rospy.loginfo("Calling UngoCorn")
            outcome = self.utils.UngoCornService()
            if outcome.success == "ERROR":
                rospy.logerr("UngoCorn failed")
                
            if self.utils.verbose: rospy.loginfo("Calling GoStow")
            outcome = self.utils.GoStowService()
            if outcome.success == "ERROR":
                rospy.logerr("GoStow failed")
        
        return 'success'


class FSM:
    def __init__(self):
        self.utils = Utils()
        self.main()

    def main(self):
        start_state = smach.StateMachine(outcomes = ['stop'])    # Outcome of Main State Machine
        start_state.userdata.find_stalk = (3, 10.0)  # a tuple of num_frames and timeout

        with start_state:

            
            smach.StateMachine.add('Finding_Cornstalk',find_cornstalk(self.utils),
                                transitions = {'success':'Insertion',
                                               'error':'stop',
                                               'next':'Finding_Cornstalk',
                                               'reposition':'stop'},
                                remapping = {'state_1_input':'find_stalk'})
            smach.StateMachine.add('Visual_Servoing',visual_servoing(self.utils),
                                transitions = {'success':'Finding_Cornstalk',
                                               'error':'stop'})
            smach.StateMachine.add('Insertion',insertion(self.utils),
                                transitions = {'success':'Finding_Cornstalk',
                                               'error':'stop'})
        
        sis = smach_ros.IntrospectionServer('server_name', start_state, '/nimo_fsm')
        sis.start()
        start_state.execute()
        sis.stop()

if __name__ == '__main__':
    rospy.init_node('nimo_fsm')
    fsm = FSM()