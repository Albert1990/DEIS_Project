#!/usr/bin/env python

import red_board
import time
from robot import Robot
from line_tracker import LineTracker
from utils import RobotPos
import math

import rospy
from std_msgs.msg import String
import cv2
import cv_bridge
from cv_bridge import CvBridgeError

goalPos = RobotPos(550,65)
firstPacket = True

def gpsInfoHandlar(data):
    global lineTracker
    global goalPos
    global firstPacket

    if firstPacket:
        lineTracker.startLineTracker()
        firstPacket = False
    receivedData = data.data
    # print('receivedData: %s' % (receivedData))
    parts = receivedData.split(',')
    robotPos = RobotPos(parts[0], parts[1])
    print('robotPosX: %d, robotPosY: %d' % (robotPos.X, robotPos.Y))
    print('goalPosX: %d, goalPosY: %d' % (goalPos.X, goalPos.Y))
    diff = math.sqrt( (robotPos.X - goalPos.X)**2 + (robotPos.Y - goalPos.Y)**2 )
    print('diff %f' % (diff))
    if diff <= 50:
       lineTracker.stopLineTracker()
  
def listener():
    print('listner started !')
    rospy.init_node('Group1Test2', anonymous=True)
    rospy.Subscriber("xy_abd", String, gpsInfoHandlar)
    rospy.spin()

robot = Robot('/dev/ttyUSB0')
lineTracker = LineTracker(robot)
listener()
