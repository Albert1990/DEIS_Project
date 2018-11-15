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
from camera import Camera

goalPos = RobotPos(550,65)
firstPacket = True
task = 'sideformation'
robotName = 'Zingo'

def gpsInfoHandlar(data):
    global lineTracker
    global goalPos
    global firstPacket

    if firstPacket:
        lineTracker.startLineTracker()
        firstPacket = False
    receivedData = data.data
    print('receivedData: %s' % (receivedData))
    parts = receivedData.split(',')
    if robotName == 'Zingo':
        myPos = RobotPos(parts[2], parts[3])
        friendPos = RobotPos(parts[0], parts[1])
    else:
        myPos = RobotPos(parts[0], parts[1])
        friendPos = RobotPos(parts[2], parts[3])
    if task == 'catchGoal':
        diff = math.sqrt( (myPos.X - goalPos.X)**2 + (myPos.Y - goalPos.Y)**2 )
        print('diff %f' % (diff))
        if diff <= 50:
            lineTracker.stopLineTracker()
            robot.stop()
    if task == 'sideformation':
        diffY = abs(friendPos.Y - myPos.Y)
        if diffY < 30:
            lineTracker.changeLane('right')
        diffX = (friendPos.X - myPos.X)
        if myPos.Y > 300:
            diffX = -diffX
        
        if diffX <= 33:
            lineTracker.stopLineTracker()
            robot.stop()
        else:
            lineTracker.startLineTracker()

        # if myPos.Y < 300:
        #     if diff <= -50:
        #         lineTracker.stopLineTracker()
        #         robot.stop()
        #     else:
        #         lineTracker.startLineTracker()
        # else:
        #     if diff > 50:
        #         lineTracker.stopLineTracker()
        #         robot.stop()
        #     else:
        #         lineTracker.startLineTracker()
        
  
def listener():
    print('listner started !')
    rospy.init_node('Group1Test2', anonymous=True)
    rospy.Subscriber("xy_abd", String, gpsInfoHandlar)
    rospy.spin()

robot = Robot('/dev/ttyUSB0')
lineTracker = LineTracker(robot)
camera = Camera()
listener()

# while True:
#     if camera.isObstacleAvailable():
#         lineTracker.stopLineTracker()
#         robot.stop()
#     else:
#         lineTracker.startLineTracker()
#     # if task == 'changeLane':
#     #     print('1')
#     # if task == 'sideformation':

#     # if task == 'catchTheGoal':
#     #     print('3')
#     time.sleep(0.05)