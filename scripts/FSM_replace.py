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

from amiga_path_planning.srv import *

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
        self.sensor_limit = 1  # change how frequent sensor swapping should be
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



    def callback(self,idk):
        if self.enable_external_mechanisms:
            if self.verbose: rospy.loginfo("Calling ControlPumps Off")
            outcome = self.ControlPumpsService("pumpsoff")
            if outcome.success == "ERROR":
                rospy.logerr("ControlPumps Off failed")
    
# State 1: Replace
class replace(smach.State):
    '''
    Replace State
    - Perform sensor replacement action
    - Bring arm to stow position at the end
    '''

    def __init__(self, utils):
        smach.State.__init__(self,
                            outcomes = ['success', 'error', 'stop'])
        self.utils = utils
    
    def execute(self, userdata):
        if self.utils.verbose: rospy.loginfo("----- Entering Replace State -----")
        rospy.set_param('pruning_status', False)
        if self.utils.enable_replacement:

            rospy.loginfo(self.utils.inserts % self.utils.sensor_limit == 0)
            rospy.loginfo(self.utils.curr_sensor_slot > 1)

            if self.utils.inserts % self.utils.sensor_limit == 0 and self.utils.curr_sensor_slot >= 1:
                rospy.logerr("Need to replace sensor")

                if self.utils.enable_manipulation:
                    # Reset the arm
                    if self.utils.verbose: rospy.loginfo("Calling GoStow")
                    outcome = self.utils.GoStowService()
                    if outcome.success == "ERROR":
                        rospy.logerr("GoStow failed")
                        return 'error'

                # If manual replacement, stop the system
                    if self.utils.sensor_replacement == "manual":
                        # TODO: Call manual replacement service
   
                
                        # Extend the linear actuator
                        if self.utils.enable_end_effector:
                            if self.utils.verbose: rospy.loginfo("Calling ActLinear Unload")
                            outcome = self.utils.ActLinearService("unload")
                            if outcome.flag == "ERROR":
                                rospy.logerr("ActLinear Unload failed")
                                return 'error'
                
                        return 'stop'
            
                    # If automatic replacement, try finding more cornstalks
                    elif self.utils.sensor_replacement == "auto":

                        # Moving to the RM slot 0 for sensor removal 
                        if self.utils.verbose: rospy.loginfo("Calling GoRM slot 0 for sensor removal")
                        outcome = self.utils.GoRMService(0)
                        if outcome.success == "ERROR":
                            rospy.logerr("GoRM SLOT 0 failed")
                            return 'error'

                        # Taking the current sensor inside out 
                        if self.utils.enable_end_effector:
                            if self.utils.verbose: rospy.loginfo("Calling ActLinear Unload")
                            outcome = self.utils.ActLinearService("unload")
                            if outcome.flag == "ERROR":
                                rospy.logerr("ActLinear Unload failed")
                                return 'error'
                
                        # Moving to a sensor slot for swapping
                        if self.utils.verbose: rospy.loginfo("Calling GoRM sensor slot for sensor loading")
                        # slot = str(self.utils.curr_sensor_slot)
                        outcome = self.utils.GoRMService(self.utils.curr_sensor_slot)
                        if outcome.success == "ERROR":
                            rospy.logerr("GoRM failed")
                            return 'error'

                        # Loading a new sensor into the gripper
                        if self.utils.enable_end_effector:
                            if self.utils.verbose: rospy.loginfo("Calling ActLinear Load")
                            outcome = self.utils.ActLinearService("load")
                            if outcome.flag == "ERROR":
                                rospy.logerr("ActLinear Load failed")
                                return 'error'
                    
                        # Moving back to Stow position
                        if self.utils.enable_manipulation:
                            # Reset the arm
                            if self.utils.verbose: rospy.loginfo("Calling GoStow")
                            outcome = self.utils.GoStowService()
                            if outcome.success == "ERROR":
                                rospy.logerr("GoStow failed")
                                return 'error'

                        self.utils.curr_sensor_slot -= 1 
                        return 'success'
            else: return 'stop'

            return 'success'



class FSM:
    def __init__(self):
        self.utils = Utils()
        self.main()

    def main(self):
        start_state = smach.StateMachine(outcomes = ['stop'])    # Outcome of Main State Machine
        start_state.userdata.find_stalk = (3, 10.0)  # a tuple of num_frames and timeout

        with start_state:

            smach.StateMachine.add('Replace',replace(self.utils),
                                transitions = {'success':'Replace',
                                               'error':'stop',
                                               'stop':'stop'})
        
        sis = smach_ros.IntrospectionServer('server_name', start_state, '/nimo_fsm')
        sis.start()
        start_state.execute()
        sis.stop()

if __name__ == '__main__':
    rospy.init_node('nimo_fsm')
    fsm = FSM()
