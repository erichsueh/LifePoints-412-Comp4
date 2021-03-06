#!/usr/bin/env python
import actionlib
import os
import roslib
import rospy
import smach
import smach_ros
import cv2
import cv_bridge
import imutils
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan, Joy, Image
from nav_msgs.msg import Odometry
import numpy as np
import random
import math
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import argparse
import imutils
import glob
import cv2
from std_srvs.srv import Empty

class Localization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Localization','Exploration'])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        #self.initOrientation
        self.orientation = 0
        self.initOrientation = 0
        self.range_ahead = 99
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

    def execute(self, userdata):
        #spread particles randomly
        #rosservice call /global_localization "{}"
        rospy.wait_for_service('global_localization')
        global_localization = rospy.ServiceProxy('global_localization',Empty)
        global_localization()
        #rotate 3 times
        self.moveTo()
        '''
        twist = Twist()
        twist.angular.z = 1
        self.cmd_vel_pub.publish(twist)
        state_change_time = 10
        rospy.Duration(10)
        
        while rospy.Time.now().to_sec() < state_change_time:
            pass
        '''
        #if localization == done:
        #clear cost map
        '''
        rospy.wait_for_service('move_base/clear_costmap')
        print('ejogfaij')
        clear_costmap = rospy.ServiceProxy('clear_costmap',Empty)
        clear_costmap()
        '''
        return 'Exploration'
        #else:
        #this else statement might get stuck in a loop?
        #return 'Localization'

    def moveTo(self):

        while (abs((self.initOrientation - (math.pi/4) + 0.1) - self.orientation) > 0.1):
            print(abs((self.initOrientation - (math.pi) + 0.1) - self.orientation))
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            
    def odom_callback(self, msg):

        self.orientation = msg.pose.pose.orientation.z

class Exploration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Localization','Exploration','Found'])
        #need to edit these waypoints
        self.found = False
        self.currentpos = [0,0,0]
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.waypoints = [
            [(-5.7, -3.1, 0.0), (0.0, 0.0, .99, 0.07)],
            [(-6.2, -0.89, 0.0), (0.0, 0.0, -0.78, 0.64)],
            [(-.46, -.09, 0.0), (0.0, 0.0, -.06, 1.0)],
            [(0.03, 2.05, 0.0), (0.0, 0.0, .7, 0.7)]
        ]
        self.waycounter = 0
        self.image = None
        self.bridge = cv_bridge.CvBridge()

        image1 = cv2.imread('marker4.png')
        image2 = cv2.imread('UAEmblem.png')

        gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

        gray1 = imutils.resize(gray1, width = int((gray1.shape[1]* 0.5)))
        gray2 = imutils.resize(gray2, width = int((gray2.shape[1] * 0.1)))

        self.template1 = cv2.Canny(gray1, 100, 200)
        (self.tH1, self.tW1) = self.template1.shape[:2]

        self.template2 = cv2.Canny(gray2, 100, 200)
        (self.tH2, self.tW2) = self.template2.shape[:2]

    def im_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
        self.found = self.matchtemplate()
        #self.cam_info_sub = rospy.Subscriber('/cv_camera/camera_info', CameraInfo, self.info_cb)

    def findClosestWaypoint(self):
        #get current pos in map
        minValue = 99
        wayIndex = None
        for i in range(len(self.waypoints)):
            distAway = math.sqrt(math.pow(self.waypoints[i][0][0] - currentpos.x, 2) + math.pow(self.waypoints[i][0][1] - currentpos.y, 2))
            if distAway < minValue:
                minValue = distAway
                wayIndex = i
            
        return wayIndex

    def closeToWaypoint(self, goal):
        #get current pos in map
        distAway = math.sqrt(math.pow(goal.target_pose.pose.position.x - self.currentpos[0], 2) + math.pow(goal.target_pose.pose.position.y - self.currentpos[1], 2))
        angleAway = goal.target_pose.pose.orientation.z - self.currentpos[2]

        if(distAway < 0.1 and angleAway < 0.1):
            return True
        else:
            return False


    def execute(self, userdata):
        self.found = False
        #do stuff here
        #have a waypoint counter
        # 0 = hasn't been to a waypoint
        #if waypoint = 0, find closest point
        #else find closest point not including waypoint you've already been to 
        self.img_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.im_callback)
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
        
        while(self.found != None):
            if(self.closeToWaypoint(goal)):
                return "Exploration"
            print(self.found)
            #if(self.lostPosition):
            #    return "Localization"
        print("ive foudn it")
        self.img_sub.unregister()
        #os.system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}")
        return "Found"

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

        
    def matchtemplate(self):
        
        while(self.image == None):
            continue
        print "have image"
        currImage = self.image
        space = np.linspace(1,0.05, 30)
        
        found1 = None
        for scale in space:
            #print "Webcam"
            
            resized = imutils.resize(currImage, width = int((currImage.shape[1]) * scale))
		    #r1 = currImage.shape[1] / float(resized.shape[1])
            r1 = 0

            if resized.shape[0] < self.tH1 or resized.shape[1] < self.tW1:
			    break

            edged = cv2.Canny(resized, 100, 200)
            result1 = cv2.matchTemplate(edged, self.template1, cv2.TM_CCOEFF_NORMED)
            (minVal1, maxVal1, _, maxLoc1) = cv2.minMaxLoc(result1)

            if found1 is None or maxVal1 > found1[0]:
			    found1 = (maxVal1, maxLoc1, r1)


        found2 = None
        for scale in space:
            
            resized = imutils.resize(currImage, width = int(currImage.shape[1] * scale))
		    #r2 = currImage.shape[1] / float(resized.shape[1])
            r2 = 0
            if resized.shape[0] < self.tH2 or resized.shape[1] < self.tW2:
			    break

            edged = cv2.Canny(resized, 50, 200)
            result2 = cv2.matchTemplate(edged, self.template2, cv2.TM_CCOEFF_NORMED)
            (minVal2, maxVal2, _, maxLoc2) = cv2.minMaxLoc(result2)

            if found2 is None or maxVal2 > found2[0]:
			    found2 = (maxVal2, maxLoc2, r2)


        print "Webcam", maxVal1,maxVal2
        
        if(maxVal1 > 0.06 or maxVal2 > 0.06):
            print('changed to true')
            self.found = True
        
class Found(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Localization','Exploration','Found','Sound'])
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.orientation = 0
        self.initOrientation = 0
        self.range_ahead = 99

        
    
    def im_callback(self, msg):
        return

    def scan_sub(self, msg):
        
        if(not math.isnan(msg.ranges[index])):
            self.range_ahead = msg.ranges[len(msg.ranges)/2]

    def odom_callback(self, msg):

        self.orientation = msg.pose.pose.orientation.z

    def moveTo(self):

        while (abs((self.initOrientation - (math.pi/4) + 0.1) - self.orientation) > 0.1):
            twist = Twist()
            twist.angular.z = -0.5
            self.cmd_vel_pub.publish(twist)

        while(self.range_ahead > 0.6):
            twist = Twist()
            twist.linear.x = 0.3
            self.cmd_vel_pub.publish(twist)

            return 'Sound' 

    def execute(self, userdata):

        #return 'Sound'

        self.orientation = 0
        self.initOrientation = 0

        self.im_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.im_callback)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        while(self.orientation == 0):
            continue

        self.initOrientation = orientation
        
        test = self.moveTo()
        self.im_sub.unregister()
        #self.scan_sub.unregister()
        self.odom_sub.unregister()
        return test

        

class Sound(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Exploration','Sound'])
        self.soundhandle = SoundClient()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.orientation = 0
        self.initOrientation = 0
        self.image = None
        self.bridge = cv_bridge.CvBridge()

        image1 = cv2.imread('marker4.png')
        image2 = cv2.imread('UAEmblem.png')

        gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

        gray1 = imutils.resize(gray1, width = int((gray1.shape[1]* 0.5)))
        gray2 = imutils.resize(gray2, width = int((gray2.shape[1] * 0.1)))

        self.template1 = cv2.Canny(gray1, 100, 200)
        (self.tH1, self.tW1) = self.template1.shape[:2]

        self.template2 = cv2.Canny(gray2, 100, 200)
        (self.tH2, self.tW2) = self.template2.shape[:2]

    def im_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)

    def odom_callback(self, msg):

        self.orientation = msg.pose.pose.orientation.z

    def checkMarker(self):

        while(self.image == None):
            continue
        print "have image"
        currImage = self.image
        space = np.linspace(1,0.05, 30)
        
        found1 = None
        for scale in space:
            print scale
            
            resized = imutils.resize(currImage, width = int((currImage.shape[1]) * scale))
		    #r1 = currImage.shape[1] / float(resized.shape[1])
            r1 = 0

            if resized.shape[0] < self.tH1 or resized.shape[1] < self.tW1:
			    break

            edged = cv2.Canny(resized, 100, 200)
            result1 = cv2.matchTemplate(edged, self.template1, cv2.TM_CCOEFF_NORMED)
            (minVal1, maxVal1, _, maxLoc1) = cv2.minMaxLoc(result1)

            if found1 is None or maxVal1 > found1[0]:
			    found1 = (maxVal1, maxLoc1, r1)


        found2 = None
        for scale in space:
            print scale
            
            resized = imutils.resize(currImage, width = int(currImage.shape[1] * scale))
		    #r2 = currImage.shape[1] / float(resized.shape[1])
            r2 = 0

            print resized.shape

            if resized.shape[0] < self.tH2 or resized.shape[1] < self.tW2:
			    break

            edged = cv2.Canny(resized, 50, 200)
            result2 = cv2.matchTemplate(edged, self.template2, cv2.TM_CCOEFF_NORMED)
            (minVal2, maxVal2, _, maxLoc2) = cv2.minMaxLoc(result2)

            if found2 is None or maxVal2 > found2[0]:
			    found2 = (maxVal2, maxLoc2, r2)


        print maxVal1
        print maxVal2
        
        if(found1 is None or maxVal1 < maxVal2):
            return 2
        else:
            return 1


    def returnToPath(self):

        while (abs((self.initOrientation + (math.pi/2) - 0.2) - self.orientation) > 0.1):
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)

    def execute(self, userdata):
        #do stuff here
        self.orientation = None
        self.initOrientation = 0
        
        self.im_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.im_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        while(self.orientation == None):
            continue

        self.initOrientation = self.orientation
        
        marker = self.checkMarker()

        if(marker == 1):
            print "marker 1"
            self.soundhandle.playWave("sound1.ogg")
        else:
            print "marker 2"
            self.soundhandle.playWave("sound2.ogg")

        #self.im_sub.unregister()
        #self.img_odom.unregister()
        self.returnToPath()

        return 'Exploration'

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
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
    client.wait_for_result()
