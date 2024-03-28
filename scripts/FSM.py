#!/usr/bin/env python
import numpy as np
import rospy
import smach
import smach_ros

from geometry_msgs.msg import Point

from stalk_detect.srv import *
from nimo_manipulation.srv import *

from stalk_detect.msg import *

VERBOSE = True

# TODO: Add error handling for services

class FindStalk(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error','next_waypoint','done'])

    def execute(self, userdata):
        if VERBOSE: rospy.loginfo("State: FindStalk")

        global visited_stalks, insertion_angle
        global GoHomeService, GetStalkService, GoCornService, ArcCornService, GetWidthService

        # Return the xArm to home
        resp = GoHomeService()

        # Find the grasp points of nearby stalks
        # resp = GetStalkService(num_frames=3, timeout=10.0)
        resp = GetStalkResponse(grasp_points=(grasp_point(position=Point(x=0.2, y=-0.5, z=0.6)), grasp_point(position=Point(x=0.2, y=-0.5, z=0.6)))) # NOTE: TEMPORARY
        
        # Iterate through stalks
        for new_point in resp.grasp_points:
            new_stalk = True
            new_point = (new_point.position.x, new_point.position.y, new_point.position.z)
            for old_point in visited_stalks:
                if np.linalg.norm(np.array(new_point) - np.array(old_point)):
                    new_stalk = False
                    break

            if new_stalk == True:
                visited_stalks.append(new_point)
                break
        
        # Move the xArm to the stalk
        current_stalk = Point(x = visited_stalks[-1][0],
                              y = visited_stalks[-1][1],
                              z = visited_stalks[-1][2])
        resp = GoCornService(grasp_point=current_stalk)
        
        width_angle = []

        # Move to minimum angle and get first width
        arc_resp = ArcCornService(relative_angle=-30)
        # width_resp = GetWidthService(num_frames=3, timeout=10.0)
        width_resp = GetWidthResponse(width=25.0+np.random.rand())  # NOTE: TEMPORARY

        width_angle.append((width_resp.width, arc_resp.absolute_angle))
        
        # Measure width and orientation
        for i in range(4):
            # Move xArm
            arc_resp = ArcCornService(relative_angle=15)

            # Get width of stalk
            # width_resp = GetWidthService(num_frames=3, timeout=10.0)
            width_resp = GetWidthResponse(width=25.0+np.random.rand())  # NOTE: TEMPORARY

            width_angle.append((width_resp.width, arc_resp.absolute_angle))
        
        # Find angle corresponding to largest width
        insertion_angle = width_angle[np.argmax(np.array(width_angle)[:, 0])][1]

        return 'next_waypoint'

# TODO: Set timeouts for services and errors if not found
def loadServices():
    global GoHomeService, GetStalkService, GoCornService, ArcCornService, GetWidthService

    # # Perception # NOTE: TEMPORARY
    # rospy.wait_for_service('get_stalk')
    # GetStalkService = rospy.ServiceProxy('get_stalk', GetStalk)
    # rospy.wait_for_service('get_width')
    # GetWidthService = rospy.ServiceProxy('get_width', GetWidth)

    # Manipulation
    rospy.wait_for_service('GoHome')
    GoHomeService = rospy.ServiceProxy('GoHome', GoHome)
    rospy.wait_for_service('GoCorn')
    GoCornService = rospy.ServiceProxy('GoCorn', GoCorn)
    rospy.wait_for_service('ArcCorn')
    ArcCornService = rospy.ServiceProxy('ArcCorn', ArcCorn)

if __name__ == "__main__":
    rospy.init_node('nimo_state_machine')

    if VERBOSE: rospy.loginfo("Starting FSM node.")

    loadServices()

    global visited_stalks, insertion_angle
    visited_stalks = []
    insertion_angle = 0

    sm = smach.StateMachine(outcomes=['next_waypoint', 'error', 'replace_sensor']) # Terminal states for SVD
    # sm = smach.StateMachine(outcomes=['ERROR']) # Terminal states for Iowa

    with sm:
        smach.StateMachine.add('FIND_STALK', FindStalk(), 
                               transitions={'error':'error',
                                            'next_waypoint':'next_waypoint',
                                            'done':'error'})
        # smach.StateMachine.add('CLEAN_CALIBRATE', CleanCalibrate(), 
        #                        transitions={'error':'error',
        #                                     'replace_sensor':'replace_sensor',
        #                                     'done':'INSERT_SENSOR'})
        # smach.StateMachine.add('Insert_Sensor', InsertSensor(), 
        #                        transitions={'error':'error',
        #                                     'replace_sensor':'replace_sensor',
        #                                     'done':'FIND_STALK'}) # NOTE: Should we check all visible stalks, or just move on?

    outcome = sm.execute()