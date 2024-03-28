#!/usr/bin/env python

import rospy
import smach
import smach_ros

from stalk_detect.srv import *
from nimo_manipulation.srv import *

VERBOSE = True

# TODO: Add error handling for services

class FindStalk():
    def __init__(self):
        smach.State.__init__(self, outcomes=['error','next_waypoint','done'])

    def execute(self, userdata):
        global GoHomeService
        # Return the xArm to home
        resp = GoHomeService()

        # Find the grasp points of nearby stalks
        
        # Iterate through stalks
        
        # xARM : GoCorn
        # Arc move loop
            # xARM : ArcMove
            # perception : GetWidth
        
        # Find angle corresponding to largest width

        return 'next_waypoint'

if __name__ == "__main__":
    rospy.init_node('nimo_state_machine')

    if VERBOSE: rospy.loginfo("Starting FSM node.")

    global GoHomeService
    rospy.wait_for_service('GoHome')
    GoHomeService = rospy.ServiceProxy('GoHome', GoHome)

    sm = smach.StateMachine(outcomes=['next_waypoint', 'error', 'replace_sensor']) # Terminal states for SVD
    # sm = smach.StateMachine(outcomes=['ERROR']) # Terminal states for Iowa

    with sm:
        smach.StateMachine.add('FIND_STALK', FindStalk(), 
                               transitions={'error':'error',
                                            'next_waypoint':'next_waypoint',
                                            'done':'CLEAN_CALIBRATE'})
        # smach.StateMachine.add('CLEAN_CALIBRATE', CleanCalibrate(), 
        #                        transitions={'error':'error',
        #                                     'replace_sensor':'replace_sensor',
        #                                     'done':'INSERT_SENSOR'})
        # smach.StateMachine.add('Insert_Sensor', InsertSensor(), 
        #                        transitions={'error':'error',
        #                                     'replace_sensor':'replace_sensor',
        #                                     'done':'FIND_STALK'}) # NOTE: Should we check all visible stalks, or just move on?

    outcome = sm.execute()