#!/usr/bin/env python
import argparse
import red_board
import time
from robot import Robot
from line_tracker import LineTracker
from utils import RobotPos
from utils import RobotStatus
from utils import Senarios
from utils import parse_senario
import math
import thread
from talker import Talker 
from listener import Listener

import rospy
from std_msgs.msg import String
import cv2
import cv_bridge
from cv_bridge import CvBridgeError

robotName = 'Zingo'
partnerPosition = RobotPos()
isLeader = False
scenario = Senarios.LINE_FOLLOW

def setSenario(sen):
    global scenario
    global robot
    scenario = sen
    robot.setStatus(RobotStatus.NOTHING)

if __name__ == "__main__":
    robot = Robot('/dev/ttyUSB0')
    lineTracker = LineTracker(robot)
    if robotName == 'Zingo':
        rospy.init_node('Group_1_Zingo', anonymous=True)
    else:
        rospy.init_node('Group_1_Ringo', anonymous=True)
    listener1 = Listener()
    talker = Talker(10)

    parser = argparse.ArgumentParser(description='Red robot program')
    parser.add_argument('--robotName', type=str, default='Zingo')
    parser.add_argument('--isLeader', type=bool, default=True)
    parser.add_argument('--senario', type=str, default='LINE_FOLLOW')
    args = parser.parse_args()
    robotName = args.robotName
    isLeader = args.isLeader
    scenario = parse_senario(args.senario)
    if robotName == 'Zingo':
        lineTracker.normalSpeed = 140
        lineTracker.changeLaneFraction = 50
        lineTracker.changeLaneSpeed = 70
        lineTracker.kp = 30
        lineTracker.kd = 80
    else:
        lineTracker.normalSpeed = 120
        lineTracker.changeLaneFraction = 70
        lineTracker.changeLaneSpeed = 100
        lineTracker.kp = 30
        lineTracker.kd = 80

    def gpsInfoHandlar(data):
        global robot
        global partnerPosition
        receivedData = data.data
        # print('receivedData: %s' % (receivedData))
        # [72 266 0.560329420160072 0 0.403142816922575;115.760363550632 410.006777530987 0.974541946123153 1 0.617652217032975;-1 -1 -1 -1 -1;-1 -1 -1 -1 -1;-1 -1 -1 -1 -1;-1 -1 -1 -1 -1;-1 -1 -1 -1 -1;-1 -1 -1 -1 -1;188.26307348859 401.237015686746 -0.98185802727514 8 0.763166075364706;-1 -1 -1 -1 -1]
        if len(receivedData) > 0:
            positions_data = receivedData.split(';')
            robot1_pos_chuncks = positions_data[5].split(' ')
            robot2_pos_chuncks = positions_data[6].split(' ')
            if robot1_pos_chuncks[1] != -1 and robot1_pos_chuncks[0] != 0 and robot2_pos_chuncks[1] != -1 and robot2_pos_chuncks[0] != -1:
                robot1Pos = RobotPos(robot1_pos_chuncks[1], robot1_pos_chuncks[0])
                robot2Pos = RobotPos(robot2_pos_chuncks[1], robot2_pos_chuncks[0])
                # print('robot1: (%d,%d)' % (robot1Pos.X, robot1Pos.Y))
                # print('robot2: (%d,%d)' % (robot2Pos.X, robot2Pos.Y))

                if robotName == 'Zingo':
                    robot.setRobotPos(robot2Pos)
                    partnerPosition = robot1Pos           
                else:
                    robot.setRobotPos(robot1Pos)
                    partnerPosition = robot2Pos



    
    # listener()
    # init gps listener
    rospy.Subscriber("josefoutput", String, gpsInfoHandlar)
    thread.start_new_thread(rospy.spin, ())
    takeOverCounter = 0
    msg_counter = 0
    while True:
        print(scenario, robot.getStatus())
        action_message = listener1.receive_action()
        feedback_message = listener1.receive_feedback()
        # print('msg_action_message:', action_message)
        # print('msg_feedback_message:', feedback_message)

        if action_message is not None:
            # print('rec action_msg:', action_message)
            if action_message.action_id == 'm':
                print('#####merge rec#####')
                if robotName == 'Ringo':
                    print('Ringo in Merge mode now !!')
                    talker.pub_feedback('m', 5, 6, 'msg 1')
                    scenario = Senarios.MERGING
                    robot.setStatus(RobotStatus.MERGING_START)
                    isLeader = False

        if feedback_message is not None:
            # print('rec feedback_msg:', feedback_message)
            if feedback_message.action_id == 'm':
                if robotName == 'Zingo' and msg_counter == 0:
                    print('Zingo in Merge mode now !!')
                    robot.setStatus(RobotStatus.MERGING_START)
                    isLeader = True
                    msg_counter += 1
                

        if scenario == Senarios.LINE_FOLLOW:
            lineTracker.turnUltrasonic(True)
            lineTracker.start()

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
                if dist < 30 and dist > 0:
                    robot.setStatus(RobotStatus.CHANGING_LANE)
                    lineTracker.stop()
                    robot.slowdown()
                    time.sleep(0.3)
                    lineTracker.changeLane('right')
            if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
                robot.setStatus(RobotStatus.NOTHING)
                lineTracker.start()

        if scenario == Senarios.SIDE_FORMATION:
            if robot.getStatus() == RobotStatus.NOTHING:
                lineTracker.start()
                time.sleep(1)
                lineTracker.stop()
                robot.setStatus(RobotStatus.CHANGING_LANE)
                lineTracker.changeLane('right')

            if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
                robot.setStatus(RobotStatus.SIDE_DRIVING)
                lineTracker.start()

            if robot.getStatus() == RobotStatus.SIDE_DRIVING:
                myPos = robot.getRobotPos()
                diffY = abs(partnerPosition.Y - myPos.Y)
                diffX = (partnerPosition.X - myPos.X)
                
                if myPos.Y > 300:
                    diffX = -diffX
                
                print("diffX:"+str(diffX))
                print("diffY:"+str(diffY))
                if diffX <= 33:
                    lineTracker.stop()
                    robot.slowdown()
                else:
                    lineTracker.start()

        if scenario == Senarios.MERGING:
            if isLeader:
                if robot.getStatus() == RobotStatus.NOTHING:
                    lineTracker.turnUltrasonic(True)
                    lineTracker.start()
                    time.sleep(1)
                    if robotName == 'Zingo':
                        talker.pub_action('m', 6, -1, 5, 'msg1')
                        print('#######message has been sent !#########')

                if robot.getStatus() == RobotStatus.MERGING_START:
                    myPos = robot.getRobotPos()
                    diffY = abs(partnerPosition.Y - myPos.Y)
                    diffX = (partnerPosition.X - myPos.X)
                    
                    if myPos.Y > 300:
                        diffX = -diffX
                    
                    print("diffX:"+str(diffX))
                    print("diffY:"+str(diffY))
                    if diffX < -60:
                        lineTracker.stop()
                        robot.setStatus(RobotStatus.CHANGING_LANE)
                        lineTracker.changeLane('left')
                    else:
                        lineTracker.start()
                if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED:
                    lineTracker.start()
                    setSenario(Senarios.LINE_FOLLOW)

            else: # follower
                if robot.getStatus() == RobotStatus.MERGING_START:
                    myPos = robot.getRobotPos()
                    diffY = abs(partnerPosition.Y - myPos.Y)
                    if diffY < 50 and diffY != 0:
                        setSenario(Senarios.LINE_FOLLOW)
                    diffX = (partnerPosition.X - myPos.X)
                    
                    if myPos.Y > 300:
                        diffX = -diffX
                    
                    # print("diffX:"+str(diffX))
                    print("diffY:"+str(diffY))
                    if diffX > 100:
                        lineTracker.start()
                    else:
                        lineTracker.stop()
                        robot.slowdown(0)
        
        if scenario == Senarios.NEW_LEADER:
            if robot.getStatus() == RobotStatus.NOTHING:
                lineTracker.start()
                time.sleep(1)
                lineTracker.stop()
                robot.setStatus(RobotStatus.CHANGING_LANE)
                lineTracker.changeLane('right')

            if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED and takeOverCounter == 0:
                takeOverCounter += 1
                robot.setStatus(RobotStatus.SIDE_DRIVING)
                lineTracker.start()

            if robot.getStatus() == RobotStatus.SIDE_DRIVING:
                myPos = robot.getRobotPos()
                diffY = abs(partnerPosition.Y - myPos.Y)
                diffX = (partnerPosition.X - myPos.X)
                
                if myPos.Y > 300:
                    diffX = -diffX
                
                # print("diffX:"+str(diffX))
                # print("diffY:"+str(diffY))
                if diffX <= 33:
                    lineTracker.stop()
                    robot.slowdown()
                else:
                    lineTracker.start()
                
                if robotName == 'Zingo':
                    talker.pub_action('m', 6, -1, 5, 'msg1')
                    print('#######message has been sent !#########')

            ## merging
            if isLeader:
                if robot.getStatus() == RobotStatus.MERGING_START:
                    myPos = robot.getRobotPos()
                    diffY = abs(partnerPosition.Y - myPos.Y)
                    diffX = (partnerPosition.X - myPos.X)
                    
                    if myPos.Y > 300:
                        diffX = -diffX
                    
                    # print("diffX:"+str(diffX))
                    # print("diffY:"+str(diffY))
                    if diffX < -60:
                        lineTracker.stop()
                        robot.setStatus(RobotStatus.CHANGING_LANE)
                        lineTracker.changeLane('left')
                    else:
                        lineTracker.start()
                if robot.getStatus() == RobotStatus.CHANGING_LANE_FINISHED and takeOverCounter == 1:
                    lineTracker.start()
                    setSenario(Senarios.LINE_FOLLOW)
                    takeOverCounter += 1

            else: # follower
                if robot.getStatus() == RobotStatus.MERGING_START:
                    myPos = robot.getRobotPos()
                    diffY = abs(partnerPosition.Y - myPos.Y)
                    if diffY < 70 and diffY != 0:
                        setSenario(Senarios.LINE_FOLLOW)
                    diffX = (partnerPosition.X - myPos.X)
                    
                    if myPos.Y > 300:
                        diffX = -diffX
                    
                    # print("diffX:"+str(diffX))
                    print("diffY:"+str(diffY))
                    if diffX > 100:
                        lineTracker.start()
                    else:
                        lineTracker.stop()
                        robot.slowdown(0)
                
        time.sleep(0.2)