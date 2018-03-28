#!/usr/bin/env python
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

class Localization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Localization','Exploration'])

    def execute(self, userdata):
        return 'Exploration'
        #do stuff here

class Exploration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Localization','Exploration','Found'])
        self.done = False

    def execute(self, userdata):
        #do stuff here
        if(not self.done):
            self.done = True
            return 'Found'

        else:
            return 'Exploration'


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

        return 'Sound'

        self.orientation = 0
        self.initOrientation = 0

        self.im_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.im_callback)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        while(self.orientation == 0):
            continue

        self.initOrientation = orientation

        return self.moveTo()

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
        print "heeelloooo???"

        print self.initOrientation + math.pi/2
        print self.orientation

        while (abs((self.initOrientation + (math.pi/2) - 0.2) - self.orientation) > 0.1):
            print "goal"
            print self.initOrientation + math.pi/2
            print "current"
            print self.orientation
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)

    def execute(self, userdata):
        #do stuff here
        self.orientation = None
        self.initOrientation = 0
        
        self.im_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.im_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        print "start execute"
        while(self.orientation == None):
            continue

        print "found orientation"
        self.initOrientation = self.orientation
        
        print "checking marker"
        marker = self.checkMarker()

        print "determined marker"
        if(marker == 1):
            print "marker 1"
            #self.soundhandle.playWave("firstsound")
        else:
            print "marker 2"
            #self.soundhandle.playWave("secondsound")

        print "hello?"
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
    sis.stop()

if __name__ == '__main__':
    main()
