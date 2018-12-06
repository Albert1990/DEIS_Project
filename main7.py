#!/usr/bin/env python

import red_board
import time
from robot import Robot
from line_tracker import LineTracker
from utils import RobotPos
from utils import RobotStatus
from utils import Senarios
import math

import rospy
from std_msgs.msg import String
import cv2
import cv_bridge
from cv_bridge import CvBridgeError

scenario = Senarios.OBSTACLE_AVOIDANCE
robot = Robot('/dev/ttyUSB0')
lineTracker = LineTracker(robot)



takeOverCounter = 0
while True:
    if scenario == Senarios.LINE_FOLLOW:
        lineTracker.start()
        pass

    if scenario == Senarios.CHANGE_LANE:
        if robot.getStatus() == RobotStatus.NOTHING:
            robot.setStatus(RobotStatus.CHANGING_LANE)
            lineTracker.start()
            time.sleep(1)
            lineTracker.stop()
            print('########## starting change lane ###########')
            lineTracker.turnUltrasonic(False)
            lineTracker.changeLane('right')
        if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
            print('++++++ change lane stopped +++++')
            robot.setStatus(RobotStatus.NOTHING)
            lineTracker.turnUltrasonic(True)
            lineTracker.start()
            time.sleep(5)
            lineTracker.stop()
            robot.stop()
    
    if scenario == Senarios.TAKE_OVER:
        if robot.getStatus() == RobotStatus.NOTHING:
            print('takeover started !')
            lineTracker.start()
            time.sleep(1)
            lineTracker.stop()
            robot.setStatus(RobotStatus.CHANGING_LANE)
            print('right change lane started !')
            lineTracker.changeLane('right')
        if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
            if takeOverCounter == 0:
                print('right change lane finished !')
                robot.setStatus(RobotStatus.NOTHING)
                lineTracker.start()
                time.sleep(3)
                lineTracker.stop()
                robot.stop()
                robot.setStatus(RobotStatus.CHANGING_LANE)
                print('left change lane started !')
                lineTracker.changeLane('left')
                takeOverCounter += 1
            else:
                print('left change lane finished !')
                robot.setStatus(RobotStatus.NOTHING)
                lineTracker.start()
                time.sleep(3)
                lineTracker.stop()
                robot.stop()
                print('robot stopped !')

    if scenario == Senarios.OBSTACLE_AVOIDANCE:
        lineTracker.turnUltrasonic(False)
        dist = robot.getUltrasonicDistance()
        print('dist:', dist)
        if robot.getStatus() == RobotStatus.NOTHING:
            print('line tracking started')
            lineTracker.start()
            if dist < 40 and dist > 0:
                print('fofo < 40')
                robot.setStatus(RobotStatus.CHANGING_LANE)
                lineTracker.stop()
                lineTracker.changeLane('right')
        if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
            robot.setStatus(RobotStatus.NOTHING)
            lineTracker.start()
                
    time.sleep(0.2)