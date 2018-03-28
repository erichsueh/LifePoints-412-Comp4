#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import math
from sensor_msgs.msg import Joy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

waypoints = [
    [(-5.7, -3.1, 0.0), (0.0, 0.0, .99, 0.07)],
    [(-6.2, -0.89, 0.0), (0.0, 0.0, -0.78, 0.64)],
    [(-.46, -.09, 0.0), (0.0, 0.0, -.06, 1.0)],
    [(0.03, 2.05, 0.0), (0.0, 0.0, .7, 0.7)]
]

class Localization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Localization','Exploration'])

    def execute(self, userdata):
        return 'Localization'
        #do stuff here

class Exploration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Localization','Exploration','Found'])

    def execute(self, userdata):
        #do stuff here
        for pose in waypoints:
            curr_pose = pose
            goal = goal_pose(pose)
            client.send_goal(goal)
        return 'Localization'
class Found(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Localization','Exploration','Found','Sound'])

    def execute(self, userdata):
        #do stuff here
        return 'Localization'

class Sound(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Exploration','Sound'])

    def execute(self, userdata):
        #do stuff here
        return 'Localization'

def main():
    rospy.init_node('Egghunt')

    sm = smach.StateMachine(outcomes=[])

    with sm:
        smach.StateMachine.add('Localization', Localization(), transitions = {'Localization':'Localization', 'Exploration':'Exploration'})
        smach.StateMachine.add('Exploration', Exploration(), transitions = {'Localization':'Localization', 'Exploration':'Exploration','Found':'Found'})
        smach.StateMachine.add('Found', Found(), transitions = {'Localization':'Localization', 'Exploration':'Exploration','Found':'Found','Sound':'Sound'})
        smach.StateMachine.add('Sound', Sound(), transitions = {'Exploration':'Exploration','Sound':'Sound'})

    sis = smach_ros.IntrospectionServer('Egghunt', sm, '/SM_ROOT')
    sis.start()
    
    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    main()
    client.wait_for_result()
