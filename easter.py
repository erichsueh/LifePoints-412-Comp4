#!/usr/bin/env python
import actionlib
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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import argparse
import imutils
import glob
import cv2

class Localization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Localization','Exploration'])

    def execute(self, userdata):
        #spread particles randomly
        #rosservice call /global_localization "{}"
        rospy.wait_for_service('global_localization')
        try:
            global_localization = rospy.ServiceProxy('global_localization',GlobalLocalization)
        except:
            print("Global Loc Failed")
        #rotate 3 times
        twist.angular.z = 30
        cmd_vel_pub.publish(twist)
        state_change_time = 30
        rospy.Duration(30)
        while rospy.Time.now() < state_change_time:
            pass
        #if localization == done:
        #clear cost map
        rospy.wait_for_service('clear_costmap')
        try:
            clear_costmap = rospy.ServiceProxy('clear_costmap',ClearCostmap)
        except:
            print("Costmap service failed!")
        
        return 'Exploration'
        #else:
        #this else statement might get stuck in a loop?
        #return 'Localization'

class Exploration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Localization','Exploration','Found'])
        #need to edit these waypoints
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.waypoints = [
            [(-5.7, -3.1, 0.0), (0.0, 0.0, .99, 0.07)],
            [(-6.2, -0.89, 0.0), (0.0, 0.0, -0.78, 0.64)],
            [(-.46, -.09, 0.0), (0.0, 0.0, -.06, 1.0)],
            [(0.03, 2.05, 0.0), (0.0, 0.0, .7, 0.7)]
        ]
        self.waycounter = 0

    def execute(self, userdata):
        #do stuff here
        #have a waypoint counter
        # 0 = hasn't been to a waypoint
        #if waypoint = 0, find closest point
        #else find closest point not including waypoint you've already been to 
        if self.waycounter == 0:
            self.waycounter == 1
            pose = self.waypoints[0]
        else:
            if waycounter <= 4:
                waycounter += 1
            else:
                waycounter = 1
            pose = self.waypoints[waycounter-1]
        curr_pose = pose
        goal = self.goal_pose(pose)
        self.client.send_goal(goal)
        #if matchtemplate worked, stop and change state to found
        #elseif lost , return localizationtion
        return 'Localization'
        #else, change waypoint counter, and return exploration
    
    def goal_pose(self,pose):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

        
    def matchtemplate(self,image):
        '''
        ap = argparse.ArgumentParser()
        ap.add_argument("-t", "--template", required=True, help="Path to template image")
        ap.add_argument("-i", "--images", required=True,
	help="Path to images where template will be matched")
        ap.add_argument("-v", "--visualize",
	help="Flag indicating whether or not to visualize each iteration")
        args = vars(ap.parse_args())
        '''
        #not constantly resizing the image here might help
        # load the image image, convert it to grayscale, and detect edges
        template = cv2.imread(image)
        template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        template = cv2.Canny(template, 50, 200)
        (tH, tW) = template.shape[:2]
        cv2.imshow("Template", template)
        # loop over the images to find the template in
        for imagePath in glob.glob(args["images"] + "/*.jpg"):
            # load the image, convert it to grayscale, and initialize the
            # bookkeeping variable to keep track of the matched region
            image = cv2.imread(imagePath)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            found = None
            # loop over the scales of the image
            for scale in np.linspace(0.2, 1.0, 20)[::-1]:
                # resize the image according to the scale, and keep track
                # of the ratio of the resizing
                resized = imutils.resize(gray, width = int(gray.shape[1] * scale))
                r = gray.shape[1] / float(resized.shape[1])
                # if the resized image is smaller than the template, then break
                # from the loop
                if resized.shape[0] < tH or resized.shape[1] < tW:
                    break
                # detect edges in the resized, grayscale image and apply template
                # matching to find the template in the image
                edged = cv2.Canny(resized, 50, 200)
                result = cv2.matchTemplate(edged, template, cv2.TM_CCOEFF)
                (_, maxVal, _, maxLoc) = cv2.minMaxLoc(result)
                # check to see if the iteration should be visualized
                if args.get("visualize", False):
                    # draw a bounding box around the detected region
                    clone = np.dstack([edged, edged, edged])
                    cv2.rectangle(clone, (maxLoc[0], maxLoc[1]),(maxLoc[0] + tW, maxLoc[1] + tH), (0, 0, 255), 2)
                    cv2.imshow("Visualize", clone)
                    cv2.waitKey(0)

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
